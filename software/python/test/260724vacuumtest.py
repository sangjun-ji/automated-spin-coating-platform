import socket
import json
import time
import serial

# ==========================================
# 1. 포트 및 통신 설정
# ==========================================
FMC_HOST = '127.0.0.1'
FMC_PORT = 50000

GRIPPER_PORT  = 'COM12'   # 그리퍼 RS-485
PIPETTE_PORT  = 'COM13'   # 피펫 & 피펫 Z축 RS-485
HOTPLATE_PORT = 'COM24'   # 핫플레이트 RS-485
RELAY_PORT    = 'COM20'   # 릴레이 보드 (진공 펌프) - 아두이노 IDE 115200과 동일

BAUD_GRIP  = 115200      
BAUD_PIP   = 38400
BAUD_RELAY = 115200       # 아두이노 Serial.begin(115200) 맞춤

P_ADDR   = 1    # 피펫 주소
P_Z_ADDR = 41   # 피펫 Z축 주소

# ==========================================
# 2. RS-485 / 핫플레이트 / 릴레이 헬퍼 함수
# ==========================================
def crc16(data):
    """그리퍼 Modbus RTU CRC16 계산"""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc.to_bytes(2, 'little')

def send_gripper_cmd(ser, command, value=0):
    """그리퍼 명령 전송"""
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
    """피펫 / 피펫 Z축 OEM 프로토콜 전송"""
    try:
        header = 0xAA
        cmd_bytes = cmd_str.encode('ascii')
        frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
        checksum = sum(frame) % 256
        frame.append(checksum)
        
        ser.write(frame)
        ser.flush()
        print(f"   └ 🧪 [피펫 RS485 (Addr:{addr})] 전송: {cmd_str}")
        time.sleep(0.5) 
    except Exception as e: 
        print(f"❌ 피펫 에러: {e}")

def calculate_checksum(packet_str):
    """핫플레이트 패킷 체크섬 계산"""
    checksum_val = sum(ord(char) for char in packet_str)
    return f"{checksum_val & 0xFF:02X}"

def set_temperature(target_temp, address="01"):
    """핫플레이트 온도 설정 패킷 생성"""
    temp_hex = f"{int(target_temp * 10):04X}"
    command_body = f"{address}DWR,01,0301,{temp_hex}"
    checksum = calculate_checksum(command_body)
    return f"\x02{command_body}{checksum}\r\n"

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
# 4. 프로세스 레시피 (hotplate / pump 키 적용)
# ==========================================
PROCESS_RECIPE = [
    {"step": "초기화 확인 및 펌프 대기", "x": None, "y": None, "z": None, "grip": 1000, "pip_z": None, "pipette": "It16000,100,2", "hotplate": 26, "pump": 0, "delay": 0.1},
    {"step": "진공 펌프 ON", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": 1, "delay": 30.0},
    {"step": "진공 펌프 OFF", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": 0, "delay": 3.0},
    
]

# ==========================================
# 5. 메인 자동화 실행 루프
# ==========================================
if __name__ == "__main__":
    print("=== 🤖 FULL PASCAL 자동화 통합 제어 시스템 가동 ===")
    
    s_grip = None
    s_pip = None
    s_hotplate = None
    s_relay = None

    try:
        # 하드웨어 시리얼 연결
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        s_hotplate = serial.Serial(HOTPLATE_PORT, baudrate=9600, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=2)
        s_relay = serial.Serial(RELAY_PORT, BAUD_RELAY, timeout=1)
        print("✅ 모든 RS-485 및 릴레이 시리얼 연결 완료!")
    except Exception as e:
        print(f"⚠️ 일부 시리얼 포트 연결 실패: {e}")

    # 초기 가드 설정 (시작하자마자 펌프 안전 OFF 명령 즉시 전송)
    if s_relay and s_relay.is_open:
        s_relay.write(b"P0\n")
        s_relay.flush()
        time.sleep(0.05)

    # 1. 피펫 Z축 상단 대피 & FMC4030 원점 잡기
    if s_pip and s_pip.is_open:
        print("\n🛡️ [안전] 피펫 Z축 최상단 안전 위치 대피...")
        send_pipette_oem(s_pip, P_Z_ADDR, "Zz50000")
        time.sleep(3.0)

    home_fmc_axis("Z", axis_num=2, speed=25.0, fall_step=5.0, direction=1)
    home_fmc_axis("X", axis_num=0, speed=30.0, fall_step=5.0, direction=2)
    home_fmc_axis("Y", axis_num=1, speed=30.0, fall_step=5.0, direction=2)
    
    if s_grip and s_grip.is_open:
        print("\n[INIT] 그리퍼 원점 초기화 탐색...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(4.0)

    print("\n✨ 모든 시스템 준비 완료! 통합 레시피 시작합니다.\n")
    time.sleep(1)

    # 2. 레시피 실행
    try:
        for idx, task in enumerate(PROCESS_RECIPE):
            print(f"\n▶ [STEP {idx+1}] {task['step']}")
            
            # --- [1] FMC4030 XYZ 절대 좌표 이동 ---
            if task.get('x') is not None:
                move_fmc_absolute("X", axis_num=0, target_pos=task['x'], speed=150.0)
                
            if task.get('y') is not None:
                move_fmc_absolute("Y", axis_num=1, target_pos=task['y'], speed=150.0)
                
            if task.get('z') is not None:
                z_target = task['z'] if task['z'] < 0 else -abs(task['z'])
                move_fmc_absolute("Z", axis_num=2, target_pos=z_target, speed=50.0)
                
            # --- [2] 그리퍼 ---
            if task.get('grip') is not None and s_grip and s_grip.is_open:
                send_gripper_cmd(s_grip, 104, task['grip'])
                
            # --- [3] 피펫 Z축 ---
            if task.get('pip_z') is not None and s_pip and s_pip.is_open:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])
                
            # --- [4] 피펫 본체 ---
            if task.get('pipette') is not None and s_pip and s_pip.is_open:
                send_pipette_oem(s_pip, P_ADDR, task['pipette'])
                
            # --- [5] 핫플레이트 ---
            if task.get('hotplate') is not None and s_hotplate and s_hotplate.is_open:
                packet = set_temperature(task['hotplate'])
                s_hotplate.write(packet.encode('ascii'))
                print(f"   └ ♨️ [핫플레이트] {task['hotplate']}°C 설정 전송")

            # --- [6] 진공 펌프 (릴레이) ---
            if task.get('pump') is not None:
                pump_val = int(task['pump']) # 0 또는 1 정수 변환
                if s_relay and s_relay.is_open:
                    cmd = f"P{pump_val}\n".encode('ascii')
                    s_relay.write(cmd)
                    s_relay.flush()  # 버퍼 강제 즉시 전송 (필수!)
                time.sleep(0.1)
                print(f"   └ 💨 [진공 펌프] {'ON (P1)' if pump_val == 1 else 'OFF (P0)'} 전송 완료!")
        
            # --- [7] Step Delay ---
            delay_time = task.get('delay', 0.1)
            if delay_time > 0:
                time.sleep(delay_time)

        print("\n🎉 모든 레시피 공정이 완료되었습니다!")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단 (Ctrl+C)")
    finally:
        # 안전 종료 시 펌프 OFF 처리
        if s_relay and s_relay.is_open:
            try:
                s_relay.write(b"P0\n")
                s_relay.flush()
                time.sleep(0.05)
                s_relay.close()
            except: pass
        if s_grip and s_grip.is_open: s_grip.close()
        if s_pip and s_pip.is_open: s_pip.close()
        if s_hotplate and s_hotplate.is_open: s_hotplate.close()
        print("🔌 모든 시리얼 포트가 안전하게 종료되었습니다.")