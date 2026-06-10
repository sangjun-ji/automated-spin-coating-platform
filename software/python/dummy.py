import serial
import time

# ==========================================
# 1. 시스템 통신 및 포트 설정
# ==========================================
GRBL_PORT = 'COM3'      # CNC 쉴드 (X, Y, CNC-Z축)
GRIPPER_PORT = 'COM9'   # 그리퍼 (RS485 - Modbus) - 포트 번호 확인 필요
PIPETTE_PORT = 'COM6'   # 피펫 & ADP-Z (RS485 - OEM)

BAUD_GRBL = 115200
BAUD_GRIP = 115200      
BAUD_PIP  = 38400

# 이동 속도 (mm/min)
FEEDRATE_XY = 4000     # X, Y축 이동 속도
FEEDRATE_Z = 2000       # CNC Z축 이동 속도
SEEK_SPEED = 800        # 호밍 탐색 속도
PULL_OFF = 5            # 호밍 후퇴 거리

# 피펫 장치 주소
P_ADDR = 1        # SP20 피펫 본체 주소
P_Z_ADDR = 41     # 피펫 전용 Z축(ADP-Z) 주소

# ==========================================
# 2. 📝 PASCAL 공정 레시피 (Recipe)
# ==========================================
# x, y : 절대 좌표 이동 거리 (mm). 현재 위치 상관없이 지정한 좌표로 이동합니다 (G90).
# z    : 상대 좌표 이동 거리 (mm). 현재 위치 기준으로 이동합니다 (CNC 쉴드 Z축, G91).
# pip_z: 피펫 전용 Z축 제어 명령어 (ADP-Z).
# pipette: 피펫 본체 흡입/토출 명령어.
# grip : 그리퍼 파지 값 (1000=개방, 688=파지 등).
# * 값을 None으로 두면 해당 장치는 해당 스텝에서 움직이지 않습니다.

PROCESS_RECIPE = [
{"step": "초기화", "x": None, "y": None, "z": None, "grip": 1000,  "pip_z": "Zz50000", "pipette": "It16000,100,2",  "delay": 2 },
{"step": "이동", "x": -415, "y": None, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
{"step": "이동", "x": None, "y": -10, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
{"step": "이동", "x": None, "y": None, "z": None, "grip": None,  "pip_z": "Zd137000,50000", "pipette": None,  "delay": 3 },
{"step": "피펫 팁 장착", "x": None, "y": None, "z": None, "grip": None, "pip_z": "Zg5000,80", "pipette": None, "delay": 3},
{"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": "Zu100000,50000", "pipette": None, "delay": 3},
{"step": "이동", "x": None, "y": -118, "z": None, "grip": None, "pip_z": None, "pipette": None,  "delay": 0.5 },
{"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": "Zd110000,50000", "pipette": None, "delay": 3.0},
{"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": "Ia5000,200,10", "delay": 3.0},
{"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": "Zu50000,50000", "pipette": None, "delay": 3.0},
 {"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": "Da5000,500,500,300", "delay": 3.0},
 {"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": "Zu30000,50000", "pipette": None, "delay": 3.0},
 {"step": "이동", "x": None, "y": -10, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
 {"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": "Dt16000,0", "delay": 3.0},
 {"step": "이동", "x": 0, "y": 0, "z": None, "grip": None, "pip_z": None, "pipette": None, "delay": 3.0},
 {"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": None, "delay": 0.5},
{"step": "이동", "x": -69, "y": None, "z": -30, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
{"step": "이동", "x": None, "y": -139, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
{"step": "이동", "x": None, "y": None, "z": 30, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
{"step": "25mm 유리 기판 파지", "x":None, "y": None, "z": None, "grip": 680, "delay": 2.0},
{"step": "이동", "x": None, "y": None, "z": -20, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
{"step": "이동", "x": None, "y": None, "z": 20, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
]


# ==========================================
# 3. 제어 함수 정의
# ==========================================

# --- [GRBL 제어 및 호밍] ---
def wait_for_hardware_alarm(ser):
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'ALARM' in line or 'Reset to continue' in line:
            return True
        time.sleep(0.01)

def wait_until_idle(ser):
    time.sleep(0.5) 
    ser.reset_input_buffer()
    while True:
        ser.write(b'?')
        status = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'Idle' in status:
            break
        time.sleep(0.05)

def send_gcode(ser, gcode):
    ser.write((gcode + '\n').encode())
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'ok' in line:
            print(f"   └ [GRBL] 명령 수신: {gcode}")
            break

def run_xy_homing(s):
    """X, Y축만 센서를 이용해 원점을 잡는 함수"""
    print("--- [PASCAL] X, Y축 하드웨어 호밍 시작 ---")
    s.reset_input_buffer()

    # 1. 하드 리밋 켜기
    s.write(b'\x18') # 소프트 리셋
    time.sleep(1)
    s.write(b'$X\n') # 알람 해제
    time.sleep(0.5)
    s.write(b'$21=1\n') # 하드 리밋 활성화
    time.sleep(0.5)

    # 2. X축 탐색 (X+ 방향으로 이동하다 센서 충돌)
    print("\n▶ [1/2] X축 센서 탐색 중...")
    s.write(f'G91 G1 X2000 F{SEEK_SPEED}\n'.encode())
    wait_for_hardware_alarm(s) # ALARM 발생 대기
    
    # 알람 후 재설정
    s.write(b'\x18')     
    time.sleep(1)
    s.write(b'$X\n')     
    time.sleep(0.5)
    s.write(b'$21=0\n') # 이동을 위해 잠시 리밋 끄기
    time.sleep(0.5)
    
    print(f"  🏃 X축 {PULL_OFF}mm 후퇴...")
    # 수정: -X5가 아니라 X-5로 작성하고 G91을 재선언합니다.
    s.write(f'G91 G1 X-{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)

    # 3. Y축 탐색을 위해 방어막 다시 켜기
    s.write(b'$21=1\n') 
    time.sleep(0.5)

    # 4. Y축 탐색
    print("\n▶ [2/2] Y축 센서 탐색 중...")
    s.write(f'G91 G1 Y2000 F{SEEK_SPEED}\n'.encode())
    wait_for_hardware_alarm(s)
    
    s.write(b'\x18')     
    time.sleep(1)
    s.write(b'$X\n')     
    time.sleep(0.5)
    s.write(b'$21=0\n')  
    time.sleep(0.5)
    
    print(f"  🏃 Y축 {PULL_OFF}mm 후퇴...")
    # 수정: 안전을 위해 G91을 다시 포함합니다.
    s.write(f'G91 G1 Y-{PULL_OFF} F500\n'.encode())
    wait_until_idle(s)

    # 영점 선언
    s.write(b'G92 X0 Y0\n')
    print("  ✅ [X축, Y축 0점 세팅 최종 완료!]")
    time.sleep(0.5)

# --- [피펫 & ADP-Z 제어 (OEM)] ---
def send_pipette_oem(ser, addr, cmd_str):
    header = 0xAA
    cmd_bytes = cmd_str.encode('ascii')
    frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
    frame.append(sum(frame) % 256) # Checksum
    ser.reset_input_buffer()
    ser.write(frame)
    print(f"   └ [RS485-OEM] 주소 {addr}: {cmd_str}")
    time.sleep(0.2)

# --- [그리퍼 제어 (Modbus)] ---
def crc16(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc.to_bytes(2, 'little')

def send_gripper_cmd(ser, command, value=0):
    SLAVE_ID = 1
    frame = bytearray([SLAVE_ID, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04])
    frame += bytearray([(command >> 8) & 0xFF, command & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    frame += crc16(frame)
    ser.write(frame)
    print(f"   └ [그리퍼] 명령: {command}, 값: {value}")


# ==========================================
# 4. 메인 시퀀스 실행 루프
# ==========================================

def main():
    try:
        # 1. 모든 시리얼 포트 연결
        print("--- 통신 포트 연결 중 ---")
        s_grbl = serial.Serial(GRBL_PORT, BAUD_GRBL, timeout=1)
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        time.sleep(2)

        # 2. X, Y축 호밍 실행
        run_xy_homing(s_grbl)

        # 3. 그리퍼 초기화 (1회성 원점 탐색)
        print("\n[INIT] 그리퍼 초기화 (원점 탐색)...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(5) 

        print("\n--- PASCAL 통합 공정 레시피 시작 ---")
        
        for idx, task in enumerate(PROCESS_RECIPE):
            print(f"\n▶ [STEP {idx+1}] {task['step']}")

            # [1] CNC 쉴드 X, Y 구동 (절대 좌표 G90)
            if task.get('x') is not None or task.get('y') is not None:
                xy_cmd = "G90 G1"
                if task.get('x') is not None: xy_cmd += f" X{task['x']}"
                if task.get('y') is not None: xy_cmd += f" Y{task['y']}"
                xy_cmd += f" F{FEEDRATE_XY}"
                send_gcode(s_grbl, xy_cmd)
                wait_until_idle(s_grbl)

            # [2] CNC 쉴드 Z 구동 (상대 좌표 G91)
            if task.get('z') is not None:
                send_gcode(s_grbl, f"G91 G1 Z{task['z']} F{FEEDRATE_Z}")
                wait_until_idle(s_grbl)

            # [3] 그리퍼 구동 (Modbus)
            if task.get('grip') is not None:
                send_gripper_cmd(s_grip, 104, task['grip'])

            # [4] 피펫 전용 Z축 (ADP-Z) 구동
            if task.get('pip_z') is not None:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])

            # [5] 피펫 본체 (SP20) 구동
            if task.get('pipette') is not None:
                send_pipette_oem(s_pip, P_ADDR, task['pipette'])

            # [6] 지연 시간 (Delay)
            if task.get('delay', 0) > 0:
                print(f"  └ {task['delay']}초 휴식 중...")
                time.sleep(task['delay'])

        print("\n--- 모든 공정 완료 ---")

    except Exception as e:
        print(f"\n❌ 시스템 오류 발생: {e}")
    finally:
        # 안전하게 모든 포트 닫기 및 방어막 해제
        if 's_grbl' in locals() and s_grbl.is_open:
            s_grbl.write(b'$21=0\n') 
            s_grbl.close()
        if 's_grip' in locals() and s_grip.is_open: s_grip.close()
        if 's_pip' in locals() and s_pip.is_open: s_pip.close()
        print("🔌 모든 포트 연결이 안전하게 종료되었습니다.")

if __name__ == "__main__":
    main()