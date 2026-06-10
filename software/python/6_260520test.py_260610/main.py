import serial
import time

# ==========================================
# 1. 시스템 설정
# ==========================================
GRBL_PORT = 'COM3'      # XYZ축 (GRBL)
GRIPPER_PORT = 'COM7'   # 그리퍼 (RS485 - Modbus)
PIPETTE_PORT = 'COM6'   # 피펫 & ADP-Z (RS485 - OEM)

BAUD_GRBL = 115200
BAUD_GRIP = 115200      
BAUD_PIP  = 38400       # SP20 및 ADP-Z 권장 속도

# 이동 속도 (mm/min)
FEEDRATE_X = 16000
FEEDRATE_Y = 4000
FEEDRATE_Z = 2000 

# 장치 주소
P_ADDR = 1        # SP20 피펫 본체 주소
P_Z_ADDR = 41     # 피펫 전용 Z축(ADP-Z) 주소

# ==========================================
# 2. 📝 공정 레시피 (Recipe)
# ==========================================
# 값을 0.0 또는 None으로 두면 해당 단계에서 그 장치는 움직이지 않습니다.
PROCESS_RECIPE = [ 
    {"step": "시스템 초기화", "x": 0.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": "Zz50000", "pipette": "It16000,100,2", "delay": 3.0},
    
    {"step": "이동", "x": 152.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": None, "pipette": None, "delay": 1.0},
    
    {"step": "이동", "x": 0.0, "y": -3.0, "z": 0.0, "grip": None, "pip_z": None, "pipette": None, "delay": 1.0},
     
    {"step": "이동", "x": 0.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": "Zd137000,50000", "pipette": None, "delay": 3.0},
    
    {"step": "피펫 팁 장착", "x": 0.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": "Zg5000,80", "pipette": None, "delay": 3.0},

    {"step": "이동", "x": 0.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": "Zu100000,50000", "pipette": None, "delay": 3.0},
    
    {"step": "이동", "x": -141.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": None, "pipette": None, "delay": 1.0},
    
    {"step": "이동", "x": 0.0, "y": -12.0, "z": 0.0, "grip": None, "pip_z": None, "pipette": None, "delay": 1.0},
      
    {"step": "이동", "x": 0.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": "Zd110000,50000", "pipette": None, "delay": 3.0},
     
    {"step": "이동", "x": 0.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": None, "pipette": "Ia5000,200,10", "delay": 3.0},
    
    {"step": "이동", "x": 0.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": "Zu50000,50000", "pipette": None, "delay": 3.0},
    
    {"step": "이동", "x": 0.0, "y": 0.0, "z": 0.0, "grip": None, "pip_z": None, "pipette": "Da5000,500,500,300", "delay": 3.0},
    
]

# ==========================================
# 3. 제어 함수 정의
# ==========================================

# --- [GRBL 제어] ---
def send_gcode(ser, gcode):
    ser.write((gcode + '\n').encode())
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'ok' in line: break

def wait_until_idle(ser):
    time.sleep(0.1)
    ser.reset_input_buffer()
    while True:
        ser.write(b'?')
        status = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'Idle' in status: break
        time.sleep(0.05)

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
        # 모든 시리얼 포트 연결
        s_grbl = serial.Serial(GRBL_PORT, BAUD_GRBL, timeout=1)
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        
        print("--- PASCAL 통합 제어 시스템 가동 ---")
        time.sleep(2)

        for idx, task in enumerate(PROCESS_RECIPE):
            print(f"\n▶ [STEP {idx+1}] {task['step']}")

            # 1. CNC 쉴드 (GRBL) 구동
            if task.get('x', 0.0) != 0.0:
                send_gcode(s_grbl, f"G91 G1 X{task['x']} F{FEEDRATE_X}")
                wait_until_idle(s_grbl)
            
            if task.get('y', 0.0) != 0.0:
                send_gcode(s_grbl, f"G91 G1 Y{task['y']} F{FEEDRATE_Y}")
                wait_until_idle(s_grbl)
                
            if task.get('z', 0.0) != 0.0:
                send_gcode(s_grbl, f"G91 G1 Z{task['z']} F{FEEDRATE_Z}")
                wait_until_idle(s_grbl)

            # 2. 그리퍼 구동 (Modbus)
            if task.get('grip') is not None:
                send_gripper_cmd(s_grip, 104, task['grip'])

            # 3. 피펫 전용 Z축 (ADP-Z) 구동
            if task.get('pip_z') is not None:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])

            # 4. 피펫 본체 (SP20) 구동
            if task.get('pipette') is not None:
                send_pipette_oem(s_pip, P_ADDR, task['pipette'])

            # 5. 지연 시간 (Delay)
            if task.get('delay', 0) > 0:
                time.sleep(task['delay'])

        print("\n--- 모든 공정 완료 ---")

    except Exception as e:
        print(f"\n❌ 시스템 오류 발생: {e}")
    finally:
        # 안전하게 모든 포트 닫기
        if 's_grbl' in locals(): s_grbl.close()
        if 's_grip' in locals(): s_grip.close()
        if 's_pip' in locals(): s_pip.close()
        print("🔌 모든 포트 연결이 종료되었습니다.")

if __name__ == "__main__":
    main()