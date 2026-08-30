import socket
import json
import time
import serial
import odrive
from odrive.utils import dump_errors

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

GRIPPER_PORT  = 'COM12'   # 그리퍼 RS-485
PIPETTE_PORT  = 'COM13'   # 피펫 & 피펫 Z축 RS-485
HOTPLATE_PORT = 'COM25'   # 핫플레이트 RS-485
RELAY_PORT    = 'COM20'   # 릴레이 보드 (진공 펌프)

BAUD_GRIP  = 115200      
BAUD_PIP   = 38400
BAUD_RELAY = 115200       

P_ADDR   = 1    # 피펫 본체 (SADP20) 주소
P_Z_ADDR = 41   # 피펫 Z축 주소

# ODrive 상수 정의
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP = 2

odrv_start_pos = 0.0

# ==========================================
# 2. RS-485 / 핫플레이트 / 릴레이 헬퍼 함수
# ==========================================
def crc16(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc.to_bytes(2, 'little')

def send_gripper_cmd(ser, command, value=0):
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
    """피펫 OEM RS485 패킷 전송 및 응답 확인 (버퍼 초기화 적용)"""
    try:
        if ser and ser.is_open:
            # 💡 [핵심 수정] 이전 장치의 응답 잔여 데이터 버퍼 제거 (통신 충돌 방지)
            ser.reset_input_buffer()

            header = 0xAA
            cmd_bytes = cmd_str.encode('ascii')
            frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
            checksum = sum(frame) % 256
            frame.append(checksum)
            
            ser.write(frame)
            ser.flush()
            print(f"   └ 🧪 [피펫 RS485 (Addr:{addr})] 전송: {cmd_str}")
            
            time.sleep(0.2)  # 슬레이브 응답 대기
            
            # 💡 수신 응답 확인
            if ser.in_waiting > 0:
                resp = ser.read_all()
                print(f"      └ 📩 [응답 (Addr:{addr})]: {resp.hex(' ')}")
            else:
                print(f"      ⚠️ [응답 (Addr:{addr})]: 수신 응답 없음 (정상 수신 여부 확인 필요)")
                
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
    """핫플레이트 현재 온도(PV) 읽기 함수"""
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
    """급속 예열 적용 및 자동 도달 확인 함수"""
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
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        client.connect((FMC_HOST, FMC_PORT))
        client.sendall(json.dumps(command_dict).encode('utf-8'))
        response_data = client.recv(1024).decode('utf-8')
        return json.loads(response_data)

def wait_for_fmc_stop(axis_num):
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
    print(f"   ├ 📐 [{axis_name}축] 절대 이동 -> 목표 좌표: {target_pos} mm (속도: {speed})")
    command = {
        "action": "move", "axis": axis_num,
        "pos": target_pos, "speed": speed, "mode": 2
    }
    response = send_fmc_command(command)
    if response.get("res_code") == 0:
        wait_for_fmc_stop(axis_num)

# ==========================================
# 4. ODrive 스핀코터 ON/OFF 비동기 제어 함수
# ==========================================
def control_spin_cooter(odrv, target_rpm):
    global odrv_start_pos

    if target_rpm > 0:
        print(f"\n🌀 [스핀코터] {target_rpm} RPM 구동 시작...")
        odrv.axis0.encoder.config.use_index = False
        
        if not odrv.axis0.encoder.is_ready:
            print("   ❌ ODrive 엔코더 미준비 상태!")
            dump_errors(odrv)
            return False

        odrv_start_pos = odrv.axis0.encoder.pos_estimate
        target_rps = target_rpm / 60.0

        odrv.axis0.controller.config.vel_limit = 100.0
        odrv.axis0.controller.config.vel_limit_tolerance = 2.0
        odrv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        odrv.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        odrv.axis0.controller.config.vel_ramp_rate = 100.0

        odrv.axis0.controller.config.vel_gain = 0.05
        odrv.axis0.controller.config.vel_integrator_gain = 0.2

        odrv.axis0.controller.input_vel = 0.0
        odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.1)

        if odrv.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("   ❌ ODrive 제어 모드 진입 실패!")
            dump_errors(odrv)
            return False

        odrv.axis0.controller.input_vel = target_rps
        print(f"   🚀 [스핀코터] {target_rpm} RPM으로 회전 가동 중")

    else:
        print(f"\n⏹️ [스핀코터] 정지 명령 수신 -> 감속 및 정밀 영점 안착 시작...")
        
        odrv.axis0.controller.input_vel = 0.0
        while abs(odrv.axis0.encoder.vel_estimate) > 0.5:
            time.sleep(0.02)

        stage2_start = time.time()
        current_pos = odrv.axis0.encoder.pos_estimate
        relative_turns = current_pos - odrv_start_pos

        nearest_target_turns = round(relative_turns)
        target_pos = odrv_start_pos + nearest_target_turns

        odrv.axis0.controller.config.pos_gain = 30.0
        odrv.axis0.controller.config.vel_gain = 0.05
        odrv.axis0.controller.config.vel_integrator_gain = 0.2

        odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        odrv.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        odrv.axis0.controller.input_pos = target_pos

        while (time.time() - stage2_start) < 1.0:
            pos_err_deg = abs(odrv.axis0.encoder.pos_estimate - target_pos) * 360.0
            cur_vel = abs(odrv.axis0.encoder.vel_estimate)
            if pos_err_deg <= 0.5 and cur_vel < 0.05:
                break
            time.sleep(0.01)

        final_deg = ((odrv.axis0.encoder.pos_estimate - odrv_start_pos) % 1.0) * 360.0
        if final_deg > 180.0: final_deg -= 360.0
        
        print(f"   ✅ [스핀코터] 원점 복귀 완료! (최종 오차: {final_deg:.2f}°)")
        odrv.axis0.requested_state = AXIS_STATE_IDLE

    return True

# ==========================================
# 5. 통합 프로세스 레시피
# ==========================================
PROCESS_RECIPE = [
    # 초기화[cite: 8]
    {"step": "초기화 확인 및 펌프 대기", "x": None, "y": None, "z": None, "grip": 1000, "pip_z": None, "pipette": "It16000,100,2", "hotplate": 0, "pump": None, "spin": None, "delay": 0.1},
      
       #pipettipe 1-1
    {"step": "이동", "x": 485.1, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y": 10.9, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},
    {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp142000,120000", "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 2},
    {"step": "피펫 흡입/장착", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zg50000,80", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
    {"step": "피펫 Z축 상승", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp34000,120000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
   
             #vial 1-1
    {"step": "이동", "x": 657, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y": 19.3, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},  
    {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp64000,120000", "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 2},
    {"step": "감지", "x":  None, "y": None, "z":None, "grip": None, "pip_z": None, "pipette":"Ld1,10000", "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.5},
    {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp104000,20000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
    {"step": "흡입", "x":  None, "y": None, "z":None, "grip": None, "pip_z": None, "pipette":"Ia15000,100,10", "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2.2},
    {"step": "피펫 Z축 상승", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp34000,120000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},   
   

]

# ==========================================
# 6. 메인 자동화 실행 루프
# ==========================================
if __name__ == "__main__":
    print("=== 🤖 FULL PASCAL 자동화 통합 제어 시스템 가동 ===")
    
    s_grip = None
    s_pip = None
    s_hotplate = None
    s_relay = None
    odrv0 = None

    # 1. 하드웨어 시리얼 및 ODrive 연결
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
    except Exception as e:
        print(f"⚠️ 일부 시리얼 포트 연결 실패: {e}")

    try:
        print("🔍 ODrive 스핀코터 연결 중...")
        odrv0 = odrive.find_any()
        if hasattr(odrv0, 'clear_errors'): 
            odrv0.clear_errors()
            
        if not odrv0.axis0.encoder.is_ready:
            print("⚙️ [ODrive] 엔코더 영점 미설치 감지 -> 자동 캘리브레이션 진행 중...")
            odrv0.axis0.encoder.config.calib_range = 0.5
            odrv0.axis0.requested_state = 7
            
            while odrv0.axis0.current_state != 1: 
                time.sleep(0.1)
            time.sleep(0.5)

        if odrv0.axis0.encoder.is_ready:
            print("✅ ODrive 스핀코터 자동 준비 완료!")
        else:
            print("❌ ODrive 엔코더 정렬 실패!")

    except Exception as e:
        print(f"⚠️ ODrive 연결 및 설정 실패: {e}")

    # 초기 가드 설정
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

    print("✨ 모든 시스템 준비 완료! 통합 레시피를 시작합니다.\n")
    time.sleep(1)

    # 4. 레시피 실행
    try:
        spin_start_time = None
        target_spin_duration = 0.0

        for idx, task in enumerate(PROCESS_RECIPE):
            print(f"\n▶ [STEP {idx+1}] {task['step']}")

            if task.get('wait_spin_time') is not None and spin_start_time is not None:
                target_wait = task['wait_spin_time']
                elapsed = time.time() - spin_start_time
                remaining = target_wait - elapsed

                if remaining > 0:
                    print(f"   ⏳ [정밀 타이밍] 스핀 회전 시작 후 {target_wait}초 시점까지 {remaining:.2f}초 추가 대기 중...")
                    time.sleep(remaining)

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
                
            # --- [7] ODrive 스핀코터 ON/OFF 제어 ---
            if task.get('spin') is not None and odrv0 is not None:
                target_rpm = task['spin']
                if target_rpm > 0:
                    spin_start_time = time.time()
                    target_spin_duration = task.get('spin_time', 45.0)
                    control_spin_cooter(odrv0, target_rpm)
                else:
                    if spin_start_time is not None:
                        elapsed = time.time() - spin_start_time
                        remaining = target_spin_duration - elapsed
                        if remaining > 0:
                            time.sleep(remaining)
                        spin_start_time = None
                    control_spin_cooter(odrv0, 0)

            # --- [8] Step Delay ---
            delay_time = task.get('delay', 0.1)
            if delay_time > 0:
                time.sleep(delay_time)

        print("\n🎉 모든 레시피 공정이 성공적으로 완료되었습니다!")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단 (Ctrl+C)")
    finally:
        if odrv0 is not None:
            try:
                odrv0.axis0.requested_state = AXIS_STATE_IDLE
            except: pass

        if s_relay and s_relay.is_open:
            try:
                s_relay.write(b"P0\n")
                s_relay.flush()
                time.sleep(0.05)
                s_relay.close()
            except: pass

        if s_hotplate and s_hotplate.is_open:
            try:
                off_packet = set_temperature(0.0)
                s_hotplate.write(off_packet.encode('ascii'))
                s_hotplate.flush()
                time.sleep(0.1)
                s_hotplate.close()
                print("♨️ [핫플레이트] 안전 히터 OFF 완료")
            except: pass

        if s_grip and s_grip.is_open: s_grip.close()
        if s_pip and s_pip.is_open: s_pip.close()
        print("🔌 모든 하드웨어 포트가 안전하게 종료되었습니다.")