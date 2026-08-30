import socket
import json
import time
import serial

# ==========================================
# 1. 포트 및 통신 설정 (FMC4030 & 그리퍼)
# ==========================================
FMC_HOST = '127.0.0.1'
FMC_PORT = 50000

GRIPPER_PORT = 'COM12'   # 그리퍼 RS-485
BAUD_GRIP    = 115200      

# ==========================================
# 2. 그리퍼 RS-485 헬퍼 함수
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

# ==========================================
# 3. FMC4030 소켓 통신 헬퍼 함수 (XYZ축)
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
# 4. XYZ & 그리퍼 테스트 프로세스 레시피
# ==========================================
PROCESS_RECIPE = [
    {"step": "그리퍼 초기화 위치 지정", "x": None, "y": None, "z": None, "grip": 1000, "delay": 1.0},

    {"step": "이동", "x": 352.7, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y": 159.2, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},
         
]


# ==========================================
# 5. 메인 자동화 실행 루프
# ==========================================
if __name__ == "__main__":
    print("=== 🤖 XYZ축 & 그리퍼 전용 제어 시스템 가동 ===")
    
    s_grip = None

    # 1. 그리퍼 시리얼 연결
    try:
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        print("✅ 그리퍼 RS-485 시리얼 연결 완료!")
    except Exception as e:
        print(f"⚠️ 그리퍼 시리얼 포트 연결 실패: {e}")

    # 2. FMC4030 XYZ축 원점 잡기 (안전을 위해 Z축 상단 이동 후 X, Y 순차 호밍)
    home_fmc_axis("Z", axis_num=2, speed=25.0, fall_step=5.0, direction=2)
    home_fmc_axis("X", axis_num=0, speed=30.0, fall_step=5.0, direction=2)
    home_fmc_axis("Y", axis_num=1, speed=30.0, fall_step=5.0, direction=2)
    
    # 3. 그리퍼 원점 초기화
    if s_grip and s_grip.is_open:
        print("\n[INIT] 그리퍼 원점 초기화 탐색...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(4.0)

    print("✨ XYZ축 및 그리퍼 준비 완료! 레시피를 시작합니다.\n")
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

            # --- [2] 그리퍼 제어 ---
            if task.get('grip') is not None and s_grip and s_grip.is_open:
                send_gripper_cmd(s_grip, 104, task['grip'])

            # --- [3] Step Delay ---
            delay_time = task.get('delay', 0.1)
            if delay_time > 0:
                time.sleep(delay_time)

        print("\n🎉 모든 동작이 성공적으로 완료되었습니다!")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단 (Ctrl+C)")
    finally:
        if s_grip and s_grip.is_open:
            s_grip.close()
        print("🔌 시리얼 포트가 안전하게 종료되었습니다.")