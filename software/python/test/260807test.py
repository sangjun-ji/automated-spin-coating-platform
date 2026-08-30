import socket
import json
import time
import serial

# ==========================================
# 0. 공정 초기화 설정 파라미터
# ==========================================
TARGET_INIT_HOTPLATE_TEMP = 29.0
HOTPLATE_BOOST_OFFSET = 0.0     

# ==========================================
# 1. 포트 및 통신 설정
# ==========================================
FMC_HOST = '127.0.0.1'
FMC_PORT = 50000

GRIPPER_PORT  = 'COM28'   # 그리퍼 RS-485
PIPETTE_PORT  = 'COM13'   # 피펫 & 피펫 Z축 RS-485
HOTPLATE_PORT = 'COM12'   # 핫플레이트 RS-485
RELAY_PORT    = 'COM20'   # 릴레이 보드 (진공 펌프)

BAUD_GRIP  = 115200      
BAUD_PIP   = 38400
BAUD_RELAY = 115200       

P_ADDR   = 1    # 피펫 본체 (SADP20) 주소
P_Z_ADDR = 41   # 피펫 Z축 주소

# ==========================================
# 2. RS-485 / 핫플레이트 / 릴레이 헬퍼 함수
# ==========================================
def crc16(data):
    """MODBUS RTU용 CRC16 계산"""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc.to_bytes(2, 'little')

def send_gripper_cmd(ser, command, value=0):
    """그리퍼 제어 패킷 전송"""
    try:
        SLAVE_ID = 1
        frame = bytearray([SLAVE_ID, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04])
        frame += bytearray([(command >> 8) & 0xFF, command & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
        frame += crc16(frame)
        ser.write(frame)
        print(f"   └ 🤏 [그리퍼] 명령: {command}, Value: {value}")
        time.sleep(0.1)
    except Exception as e:
        print(f"❌ 그리퍼 에러: {e}")

def send_pipette_oem(ser, addr, cmd_str):
    """피펫 OEM RS485 패킷 전송 및 응답 확인"""
    try:
        if ser and ser.is_open:
            ser.reset_input_buffer()

            header = 0xAA
            cmd_bytes = cmd_str.encode('ascii')
            frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
            checksum = sum(frame) % 256
            frame.append(checksum)
            
            ser.write(frame)
            ser.flush()
            print(f"   └ 🧪 [피펫 RS485 (Addr:{addr})] 전송: {cmd_str}")
            
            time.sleep(0.2)
            
            if ser.in_waiting > 0:
                resp = ser.read_all()
                print(f"      └ 📩 [응답 (Addr:{addr})]: {resp.hex(' ')}")
            else:
                print(f"      ⚠️ [응답 (Addr:{addr})]: 수신 응답 없음")
                
            time.sleep(0.3)
    except Exception as e: 
        print(f"❌ 피펫 통신 에러: {e}")

def calculate_checksum(packet_str):
    """템코 T50 HSUM 8비트 체크섬 계산"""
    checksum_val = sum(ord(char) for char in packet_str)
    return f"{checksum_val & 0xFF:02X}"

def set_temperature(target_temp, address="01"):
    """핫플레이트 목표 온도(SV) 설정 패킷 생성"""
    temp_hex = f"{int(target_temp * 10):04X}"
    command_body = f"{address}DWR,01,0301,{temp_hex}"
    checksum = calculate_checksum(command_body)
    return f"\x02{command_body}{checksum}\r\n"

def get_current_temperature(ser, address="01"):
    """핫플레이트 현재 온도(PV) 읽기"""
    try:
        command_body = f"{address}DRS,01,0001"
        checksum = calculate_checksum(command_body)
        packet = f"\x02{command_body}{checksum}\r\n"
        
        ser.reset_input_buffer()
        ser.write(packet.encode('ascii'))
        time.sleep(0.2)
        
        response = ser.read_all().decode('ascii', errors='ignore')
        
        if "DRS,OK" in response:
            clean_resp = response.strip('\x02\x03\r\n ')
            parts = clean_resp.split(',')
            
            if len(parts) >= 3 and parts[1] == "OK":
                raw_hex_temp = parts[2][:4]
                curr_temp = int(raw_hex_temp, 16) / 10.0
                return curr_temp
                        
    except Exception as e:
        print(f"\n❌ [핫플레이트 통신 에러]: {e}")
        
    return None

def wait_for_hotplate(ser, target_temp, boost_offset=10.0, timeout=1800, check_interval=2.0):
    """급속 예열 및 자동 목표 온도 도달 확인"""
    boost_temp = target_temp + boost_offset
    print(f"\n🚀 [핫플레이트] 급속 예열 동작 개시!")
    print(f"   ├ 목표 온도: {target_temp:.1f}°C")
    
    set_packet = set_temperature(boost_temp)
    ser.write(set_packet.encode('ascii'))
    time.sleep(0.2)
    
    start_time = time.time()
    
    while True:
        curr_temp = get_current_temperature(ser)
        
        if curr_temp is not None:
            print(f"   ⏳ [급속 예열 중...] 실제 온도: {curr_temp:.1f}°C / 목표: {target_temp:.1f}°C")
            
            if curr_temp >= target_temp:
                print(f"\n✅ [핫플레이트] 목표 온도 ({target_temp:.1f}°C) 도달 확인!")
                reset_packet = set_temperature(target_temp)
                ser.write(reset_packet.encode('ascii'))
                time.sleep(0.2)
                break
        else:
            print("   ⏳ [핫플레이트] 실제 온도 수신 대기 중...")

        if time.time() - start_time > timeout:
            raise TimeoutError("핫플레이트 예열 시간 초과")

        time.sleep(check_interval)

# ==========================================
# 3. FMC4030 소켓 통신 헬퍼 함수
# ==========================================
def send_fmc_command(command_dict):
    """FMC4030 모션 컨트롤러 명령 소켓 전송"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        client.connect((FMC_HOST, FMC_PORT))
        client.sendall(json.dumps(command_dict).encode('utf-8'))
        response_data = client.recv(1024).decode('utf-8')
        return json.loads(response_data)

def wait_for_fmc_stop(axis_num):
    """축 이동 완료 상태 체크"""
    time.sleep(0.3)
    print("   ⏳ 구동 중", end="", flush=True)
    while True:
        try:
            resp = send_fmc_command({"action": "check_stop", "axis": axis_num})
            if resp.get("is_stop") == 1:
                curr_pos = resp.get("current_pos", 0.0)
                print(f" [완료! (현재 위치: {curr_pos:.2f} mm)]")
                break
            else:
                print(".", end="", flush=True)
                time.sleep(0.2)
        except Exception as e:
            print(f"\n❌ 상태 확인 에러: {e}")
            break
    time.sleep(0.2)

def home_fmc_axis(axis_name, axis_num, speed=10.0, fall_step=5.0, direction=2):
    """FMC4030 축 별 원점 호밍"""
    print(f"\n▶ [{axis_name}축] FMC4030 원점 호밍 시작...")
    command = {
        "action": "home", "axis": axis_num, "speed": speed,
        "acc_dec": 100.0, "fall_step": fall_step, "direction": direction
    }
    response = send_fmc_command(command)
    if response.get("res_code") == 0:
        wait_for_fmc_stop(axis_num)
        print(f"✅ [{axis_name}축] 원점 잡기 완료!")

def move_fmc_absolute(axis_name, axis_num, target_pos, speed=20.0):
    """FMC4030 절대 좌표 이동"""
    print(f"   ├ 📐 [{axis_name}축] 절대 이동 -> 목표 좌표: {target_pos} mm (속도: {speed})")
    command = {
        "action": "move", "axis": axis_num,
        "pos": target_pos, "speed": speed, "mode": 2
    }
    response = send_fmc_command(command)
    if response.get("res_code") == 0:
        wait_for_fmc_stop(axis_num)

# ==========================================
# 4. 통합 프로세스 레시피 (Spin 제거)
# ==========================================
PROCESS_RECIPE = [
    # 초기화
    {"step": "초기화 확인 및 펌프 대기", "x": None, "y": None, "z": None, "grip": 1000, "pip_z": None, "pipette": "It16000,100,2", "hotplate": 0, "pump": None, "delay": 0.1},


    {"step": "이동", "x": 474, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y": 6.5, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},
    {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp154000,120000", "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 2},
    {"step": "피펫 흡입/장착", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zg50000,80", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
    {"step": "피펫 Z축 상승", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp34000,120000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
   

  
    {"step": "이동", "x": 601.5, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y": 31.5, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},
    {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp78000,120000", "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 2},
    {"step": "감지", "x":  None, "y": None, "z":None, "grip": None, "pip_z": None, "pipette":"Ld1,10000", "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp119000,20000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
    {"step": "흡입", "x":  None, "y": None, "z":None, "grip": None, "pip_z": None, "pipette":"Ia15000,100,10", "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2.2},
    {"step": "피펫 Z축 상승", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp34000,120000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},       
    {"step": "분사", "x":  None, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": "Da15000,50,150,80", "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},     


    {"step": "이동", "x": 474, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y": 6.5, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},
    {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp105000,120000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
       

    {"step": "해제", "x":  None, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": "Dt16000,0", "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 1},
                     
]

# ==========================================
# 5. 메인 자동화 실행 루프
# ==========================================
if __name__ == "__main__":
    print("=== 🤖 FULL PASCAL 자동화 통합 제어 시스템 가동 (스핀코터 제외) ===")
    
    s_grip = None
    s_pip = None
    s_hotplate = None
    s_relay = None

    # 1. 하드웨어 시리얼 연결
    try:
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        
        s_hotplate = serial.Serial(
            HOTPLATE_PORT, 
            baudrate=9600, 
            bytesize=serial.SEVENBITS, 
            parity=serial.PARITY_EVEN, 
            stopbits=serial.STOPBITS_ONE, 
            timeout=2
        )
        
        s_relay = serial.Serial(RELAY_PORT, BAUD_RELAY, timeout=1)
        print("✅ 모든 RS-485 및 릴레이 시리얼 연결 완료!")
        print("⏳ 릴레이 보드 MCU 부팅 및 안정화 대기 중 (3초)...")
        time.sleep(3.0)  # 릴레이 보드 리셋 후 부팅 안정화 대기
    except Exception as e:
        print(f"⚠️ 일부 시리얼 포트 연결 실패: {e}")

    # 초기 가드 설정 (진공 펌프 OFF)
    if s_relay and s_relay.is_open:
        s_relay.write(b"P0\n")
        s_relay.flush()
        time.sleep(0.05)

    # 2. 피펫 Z축 상단 대피 & FMC4030 원점 잡기
    if s_pip and s_pip.is_open:
        print("\n🛡️ [안전] 피펫 Z축 최상단 안전 위치 대피...")
        send_pipette_oem(s_pip, P_Z_ADDR, "Zz50000")
        time.sleep(3.0)

    home_fmc_axis("Z", axis_num=2, speed=25.0, fall_step=5.0, direction=2)
    home_fmc_axis("X", axis_num=0, speed=30.0, fall_step=5.0, direction=2)
    home_fmc_axis("Y", axis_num=1, speed=30.0, fall_step=5.0, direction=2)
    
    if s_grip and s_grip.is_open:
        print("\n[INIT] 그리퍼 원점 초기화 탐색...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(4.0)

    # 3. 핫플레이트 급속 예열 부스팅 단계
    if s_hotplate and s_hotplate.is_open:
        wait_for_hotplate(
            ser=s_hotplate, 
            target_temp=TARGET_INIT_HOTPLATE_TEMP, 
            boost_offset=HOTPLATE_BOOST_OFFSET
        )
    else:
        print(f"\n⚠️ 핫플레이트 시리얼 미연결로 예열 로직을 생략합니다.")

    print("\n✨ 모든 시스템 준비 완료! 레시피를 시작합니다.\n")
    time.sleep(1)

    # 4. 레시피 실행
    try:
        for idx, task in enumerate(PROCESS_RECIPE):
            print(f"\n▶ [STEP {idx+1}] {task['step']}")

            # --- [1] FMC4030 XYZ 절대 좌표 이동 ---
            if task.get('x') is not None:
                move_fmc_absolute("X", axis_num=0, target_pos=task['x'], speed=150.0)
                
            if task.get('y') is not None:
                move_fmc_absolute("Y", axis_num=1, target_pos=task['y'], speed=150.0)
                
            if task.get('z') is not None:
                move_fmc_absolute("Z", axis_num=2, target_pos=task['z'], speed=50.0)

            # --- [2] 그리퍼 ---
            if task.get('grip') is not None and s_grip and s_grip.is_open:
                send_gripper_cmd(s_grip, 104, task['grip'])
                
            # --- [3] 피펫 Z축 (Addr: 41) ---
            if task.get('pip_z') is not None and s_pip and s_pip.is_open:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])
                
            # --- [4] 피펫 본체 SADP20 (Addr: 1) ---
            if task.get('pipette') is not None and s_pip and s_pip.is_open:
                send_pipette_oem(s_pip, P_ADDR, task['pipette'])
                
            # --- [5] 핫플레이트 ---
            if task.get('hotplate') is not None and s_hotplate and s_hotplate.is_open:
                packet = set_temperature(task['hotplate'])
                s_hotplate.write(packet.encode('ascii'))
                print(f"   └ ♨️ [핫플레이트] {task['hotplate']}°C 설정 전송")

            # --- [6] 진공 펌프 ---
            if task.get('pump') is not None:
                pump_val = int(task['pump']) 
                if s_relay and s_relay.is_open:
                    cmd = f"P{pump_val}\n".encode('ascii')
                    s_relay.write(cmd)
                    s_relay.flush()  
                time.sleep(0.1)
                print(f"   └ 💨 [진공 펌프] {'ON (P1)' if pump_val == 1 else 'OFF (P0)'} 전송 완료!")

            # --- [7] Step Delay ---
            delay_time = task.get('delay', 0.1)
            if delay_time > 0:
                time.sleep(delay_time)

        print("\n🎉 모든 레시피 공정이 성공적으로 완료되었습니다!")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단 (Ctrl+C)")
    finally:
        # 안전 종료 처리
        if s_relay and s_relay.is_open:
            try:
                s_relay.write(b"P0\n")
                s_relay.flush()
                time.sleep(0.05)
                s_relay.close()
                print("💨 [진공 펌프] 안전 OFF 완료")
            except Exception as e:
                print(f"⚠️ 릴레이 종료 에러: {e}")

        if s_hotplate and s_hotplate.is_open:
            try:
                off_packet = set_temperature(0.0)
                s_hotplate.write(off_packet.encode('ascii'))
                s_hotplate.flush()
                time.sleep(0.1)
                s_hotplate.close()
                print("♨️ [핫플레이트] 안전 히터 OFF 완료")
            except Exception as e:
                print(f"⚠️ 핫플레이트 종료 에러: {e}")

        if s_grip and s_grip.is_open: s_grip.close()
        if s_pip and s_pip.is_open: s_pip.close()
        print("🔌 모든 하드웨어 포트가 안전하게 종료되었습니다.")