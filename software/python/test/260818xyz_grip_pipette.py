import socket
import json
import time
import serial

# ==========================================
# 1. 포트 및 통신 설정
# ==========================================
FMC_HOST = '127.0.0.1'
FMC_PORT = 50000

GRIPPER_PORT = 'COM12'   # 그리퍼 RS-485
PIPETTE_PORT = 'COM13'   # 피펫 & 피펫 Z축 RS-485

BAUD_GRIP = 115200      
BAUD_PIP  = 38400

P_ADDR   = 1    # 피펫 본체 (SADP20) 주소
P_Z_ADDR = 41   # 피펫 Z축 주소

# ==========================================
# 2. 그리퍼 & 피펫 RS-485 헬퍼 함수
# ==========================================
def crc16(data):
    """그리퍼 통신용 CRC16 계산"""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc.to_bytes(2, 'little')

def send_gripper_cmd(ser, command, value=0):
    """그리퍼 RS-485 명령 전송 함수"""
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
            
            time.sleep(0.2)  # 슬레이브 응답 대기
            
            if ser.in_waiting > 0:
                resp = ser.read_all()
                print(f"      └ 📩 [응답 (Addr:{addr})]: {resp.hex(' ')}")
            else:
                print(f"      ⚠️ [응답 (Addr:{addr})]: 수신 응답 없음 (정상 수신 여부 확인 필요)")
                
            time.sleep(0.3)
    except Exception as e: 
        print(f"❌ 피펫 통신 에러: {e}")

# ==========================================
# 3. FMC4030 XYZ 축 소켓 통신 헬퍼 함수
# ==========================================
def send_fmc_command(command_dict):
    """FMC32 서버에 소켓 명령 전송"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        client.connect((FMC_HOST, FMC_PORT))
        client.sendall(json.dumps(command_dict).encode('utf-8'))
        response_data = client.recv(1024).decode('utf-8')
        return json.loads(response_data)

def wait_for_fmc_stop(axis_num):
    """FMC4030 축 정지 완료 대기"""
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
    """FMC4030 축 원점 복귀(Homing)"""
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
    """FMC4030 축 절대 좌표 이동"""
    print(f"   ├ 📐 [{axis_name}축] 절대 이동 -> 목표 좌표: {target_pos} mm (속도: {speed})")
    command = {
        "action": "move", "axis": axis_num,
        "pos": target_pos, "speed": speed, "mode": 2
    }
    response = send_fmc_command(command)
    if response.get("res_code") == 0:
        wait_for_fmc_stop(axis_num)

# ==========================================
# 4. 프로세스 레시피 (X, Y, Z, Grip, Pipette만 제어)
# ==========================================
PROCESS_RECIPE = [

          # cell plate 1-1
       {"step": "그리퍼 초기화 위치 지정", "x": None, "y": None, "z": None, "grip": 1000, "delay": 1.0},
       {"step": "이동", "x": 512.5, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
       {"step": "이동", "x": None, "y": 26, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},
       {"step": "그리퍼", "x": None, "y": None, "z": None, "grip": 650, "delay": 0.3},
       {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 110.2, "grip": None, "delay": 0.3},
       {"step": "그리퍼", "x": None, "y": None, "z": None, "grip": 520, "delay": 0.3},
       {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 10, "grip": None, "delay": 0.1},  
   
       # gripper spincoater location
       {"step": "이동", "x": 350.7, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
       {"step": "이동", "x": None, "y": 81.4, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},
       {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 69.3, "grip": None, "delay": 0.5},
       {"step": "그리퍼", "x": None, "y": None, "z": None, "grip": 675, "delay": 0.5},
       {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 10, "grip": None, "delay": 0.1}, 
   
       # 진공 펌프 on
       {"step": "진공 펌프 ON", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": 1, "spin": None, "delay": 1},  
   
   
       #pipettipe 1-1
       {"step": "이동", "x": 485.8, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
       {"step": "이동", "x": None, "y": 10.3, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},
       {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp142000,120000", "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 2},
       {"step": "피펫 흡입/장착", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zg50000,80", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
       {"step": "피펫 Z축 상승", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp34000,120000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
   
       #vial 1-1
       {"step": "이동", "x": 657, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
       {"step": "이동", "x": None, "y": 19.3, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},  
       {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp64000,120000", "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 2},
       {"step": "감지", "x":  None, "y": None, "z":None, "grip": None, "pip_z": None, "pipette":"Ld1,10000", "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
       {"step": "피펫 Z축 하강", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp104000,20000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},
       {"step": "흡입", "x":  None, "y": None, "z":None, "grip": None, "pip_z": None, "pipette":"Ia15000,100,10", "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2.2},
       {"step": "피펫 Z축 상승", "x":  None, "y": None, "z":None, "grip": None, "pip_z": "Zp34000,120000", "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 2},   
   
        #pipette location on spincoater
       {"step": "이동", "x": 352.8, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
       {"step": "이동", "x": None, "y": 159.8, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},    
   
   ]
# ==========================================
# 5. 메인 자동화 실행 루프
# ==========================================
if __name__ == "__main__":
    print("=== 🤖 XYZ / 그리퍼 / 피펫 제어 시스템 가동 ===")
    
    s_grip = None
    s_pip = None

    # 1. 하드웨어 시리얼 연결
    try:
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        print("✅ 그리퍼 및 피펫 시리얼 연결 완료!")
    except Exception as e:
        print(f"⚠️ 시리얼 포트 연결 실패: {e}")

    # 2. 피펫 Z축 상단 대피 & FMC4030 원점 잡기
    if s_pip and s_pip.is_open:
        print("\n🛡️ [안전] 피펫 Z축 최상단 안전 위치 대피...")
        send_pipette_oem(s_pip, P_Z_ADDR, "Zz50000")
        time.sleep(3.0)

    # FMC4030 XYZ 원점 복귀
    home_fmc_axis("Z", axis_num=2, speed=25.0, fall_step=5.0, direction=2)
    home_fmc_axis("X", axis_num=0, speed=30.0, fall_step=5.0, direction=2)
    home_fmc_axis("Y", axis_num=1, speed=30.0, fall_step=5.0, direction=2)
    
    # 그리퍼 원점 복귀
    if s_grip and s_grip.is_open:
        print("\n[INIT] 그리퍼 원점 초기화 탐색...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(4.0)

    print("\n✨ 모든 시스템 준비 완료! 레시피를 시작합니다.\n")
    time.sleep(1)

    # 3. 레시피 실행
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

            # --- [5] Step Delay ---
            delay_time = task.get('delay', 0.1)
            if delay_time > 0:
                time.sleep(delay_time)

        print("\n🎉 모든 레시피 공정이 성공적으로 완료되었습니다!")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단 (Ctrl+C)")
    finally:
        if s_grip and s_grip.is_open: s_grip.close()
        if s_pip and s_pip.is_open: s_pip.close()
        print("🔌 모든 하드웨어 포트가 안전하게 종료되었습니다.")