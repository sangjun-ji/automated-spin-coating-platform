import serial
import time
import sys

# ==========================================
# 1. 시스템 통신 및 포트 설정
# ==========================================
GRBL_PORT = 'COM3'       
GRIPPER_PORT = 'COM12'   
PIPETTE_PORT = 'COM13'   

BAUD_GRBL = 115200
BAUD_GRIP = 115200      
BAUD_PIP  = 38400

# 이동 속도 (mm/min)
FEEDRATE_XY = 10000     
FEEDRATE_Z = 2000      
SEEK_SPEED_Z = 600      
SEEK_SPEED_XY = 6000    # X, Y축 초당 10cm 속도로 시원하게 질주!
PULL_OFF = 5           

P_ADDR = 1        
P_Z_ADDR = 41     

# ==========================================
# 2. 📝 8단계 컴팩트 테스트 레시피
# ==========================================
PROCESS_RECIPE = [
   {"step": "초기화", "x": None, "y": None, "z": None, "grip": 1000,  "pip_z": "Zz50000", "pipette": "It16000,100,2",  "delay": 2 },
    {"step": "이동", "x": -97, "y": None, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
    {"step": "이동", "x": None, "y": -76, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
    {"step": "이동", "x": None, "y": None, "z": None, "grip": None,  "pip_z": "Zd137000,50000", "pipette": None,  "delay": 3 },
    {"step": "피펫 팁 장착", "x": None, "y": None, "z": None, "grip": None, "pip_z": "Zg5000,80", "pipette": None, "delay": 3},
    {"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": "Zu100000,50000", "pipette": None, "delay": 3},
    {"step": "이동", "x": -183, "y": None, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
    {"step": "이동", "x": None, "y": -128, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },    
    {"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": "Zd115000,50000", "pipette": None, "delay": 3.0},
    {"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": "Ia5000,200,10", "delay": 3.0},
    {"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": "Zu105000,50000", "pipette": None, "delay": 3.0},
    {"step": "이동", "x": -313, "y": None, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
    {"step": "이동", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": "Da50000,500,500,300", "delay": 3.0},
    {"step": "이동", "x": None, "y": -49, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },   
    {"step": "이동", "x": None, "y": None, "z": 208, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 }, 
    {"step": "25mm 유리 기판 파지", "x": None, "y": None, "z": None, "grip": 660, "pip_z": None, "pipette": None, "delay": 2.0},
    {"step": "이동", "x": None, "y": None, "z": -30, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 }, 
    {"step": "이동", "x": -443, "y": None, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 },
    {"step": "이동", "x": None, "y": None, "z": 14, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 }, 
    {"step": "25mm 유리 기판 파지", "x": None, "y": None, "z": None, "grip": 720, "pip_z": None, "pipette": None, "delay": 2.0},
    {"step": "이동", "x": None, "y": None, "z": -100, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.5 }, 
    
]

# ==========================================
# 3. 제어 함수 정의
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
    SLAVE_ID = 1
    frame = bytearray([SLAVE_ID, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04])
    frame += bytearray([(command >> 8) & 0xFF, command & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    frame += crc16(frame)
    ser.write(frame)
    print(f"   └ [그리퍼] 명령: {command}, value: {value}")
    time.sleep(0.1)

def send_pipette_oem(ser, addr, cmd_str):
    header = 0xAA
    cmd_bytes = cmd_str.encode('ascii')
    frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
    frame.append(sum(frame) % 256) 
    ser.reset_input_buffer()
    ser.write(frame)
    print(f"   └ [RS485-OEM] 주소 {addr}: {cmd_str}")
    time.sleep(0.2)

def wait_for_hardware_alarm(ser, filter_noise=False):
    if filter_noise:
        time.sleep(0.1) 
        ser.reset_input_buffer()

    timeout_start = time.time()
    while True:
        if ser.in_waiting > 0:
            raw_data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            # 💡 [그물망 보강] GRBL의 상태 메시지인 'State:Alarm'과 'ALARM'을 모두 쌍으로 감시합니다.
            if any(keyword in raw_data for keyword in ['ALARM', 'Alarm', 'Reset', 'error:', 'State:Alarm']):
                return True
                
        # 🛡️ [안전 최우선 복구] 15초 동안 센서 신호가 안 오면 장비를 강제 폭파(종료)시킵니다.
        if time.time() - timeout_start > 15.0: 
            print("\n🚨 [위험] Y축 센서 물리 충돌 신호 포착 실패! 장비 안전을 위해 프로세스를 강제 중단합니다.")
            sys.exit(1) # 쾅! 다음 명령 안 던지고 그 자리에서 파이썬을 즉시 완전히 종료합니다.
            
        time.sleep(0.005)

def wait_until_idle(ser):
    time.sleep(0.2) 
    while True:
        ser.write(b'?')
        status = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'Idle' in status:
            break
        time.sleep(0.05)

def send_gcode(ser, gcode):
    ser.reset_input_buffer()
    ser.write((gcode + '\n').encode())
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'ok' in line:
            break
    print(f"   └ [GRBL] 이동 시작 ➔ {gcode.strip()}")
    wait_until_idle(ser)
    print(f"   ✅ [GRBL] 목적지 도달 완료")

def run_xyz_homing_hybrid(s):
    print("--- [PASCAL] 하드웨어 충돌 감지형 3축 통합 호밍 시작 ---")
    s.reset_input_buffer()
    
    s.write(b'\x18')      
    time.sleep(1.5)       
    s.write(b'$X\n')      
    time.sleep(0.5)
    s.write(b'$21=1\n')   
    time.sleep(0.5)
    s.reset_input_buffer()

    # --------------------------------------------------
    # [1단계] Z축 상단 센서 탐색
    # --------------------------------------------------
    print("\n▶ [1/3] Z축 상단 센서 탐색 중 (위로 상승)...")
    s.write(f'G91 G1 Z-2000 F{SEEK_SPEED_Z}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True) 
    print("💥 Z축 물리적 센서 충돌 감지 완료!")
        
    time.sleep(0.1) 
    s.write(b'\x18')     
    time.sleep(1)
    s.write(b'$X\n')     
    time.sleep(0.5)
    s.write(b'$21=0\n')   
    time.sleep(0.5)
    
    print(f"   🏃 Z축 {PULL_OFF}mm 하강 후퇴...")
    s.write(f'G91 G1 Z{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)

    s.write(b'G92 Z0\n')
    print("   ✅ [Z축 상단 0점 세팅 완료]")
    time.sleep(0.5)

    # --------------------------------------------------
    # [2단계] X축 탐색 시퀀스
    # --------------------------------------------------
    print("\n▶ [2/3] X축 센서 탐색 중...")
    s.write(b'\x18')
    time.sleep(1.0)
    s.write(b'$X\n')
    time.sleep(0.5)
    s.write(b'$21=1\n') 
    time.sleep(0.5)
    s.reset_input_buffer()

    s.write(f'G91 G1 X2000 F{SEEK_SPEED_XY}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True) 
    print("💥 X축 물리적 센서 충돌 감지 완료!")
    
    time.sleep(0.1) 
    s.write(b'\x18')     
    time.sleep(1)
    s.write(b'$X\n')     
    time.sleep(0.5)
    s.write(b'$21=0\n') 
    time.sleep(0.5)
    
    print(f"   🏃 X축 {PULL_OFF}mm 후퇴...")
    s.write(f'G91 G1 X-{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)

    # --------------------------------------------------
    # [3단계] Y축 탐색 시퀀스 (💡 초고속 구동 대비 2초 뇌세척 딜레이 반영)
    # --------------------------------------------------
    print("\n▶ [3/3] Y축 센서 탐색 중...")
    s.write(b'\x18')
    time.sleep(2.0)  # 💡 아두이노가 이전 알람을 완벽히 소화할 수 있도록 2초간 홀드!
    s.write(b'$X\n')
    time.sleep(0.5)
    s.write(b'$21=1\n') 
    time.sleep(0.5)
    s.reset_input_buffer()

    s.write(f'G91 G1 Y2000 F{SEEK_SPEED_XY}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True)
    print("💥 Y축 물리적 센서 충돌 감지 완료!")
    
    time.sleep(0.1) 
    s.write(b'\x18')     
    time.sleep(1)
    s.write(b'$X\n')     
    time.sleep(0.5)
    s.write(b'$21=0\n')  
    time.sleep(0.5)
    
    print(f"   🏃 Y축 {PULL_OFF}mm 후퇴...")
    s.write(f'G91 G1 Y-{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)

    s.write(b'G92 X0 Y0\n')
    print("   ✅ [X축, Y축 0점 세팅 최종 완료!]")
    time.sleep(0.5)

# ==========================================
# 4. 메인 시퀀스 통합 실행 루프
# ==========================================
def main():
    try:
        print("--- 통신 포트 연결 중 ---")
        s_grbl = serial.Serial(GRBL_PORT, BAUD_GRBL, timeout=1)
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        time.sleep(2)

        # 3축 하이브리드 호밍 연동
        run_xyz_homing_hybrid(s_grbl)

        print("\n[INIT] 그리퍼 초기화 (원점 탐색)...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(5) 

        print("\n--- PASCAL 통합 공정 레시피 시작 ---")
        for idx, task in enumerate(PROCESS_RECIPE):
            print(f"\n▶ [STEP {idx+1}] {task['step']}")
            
            if task.get('x') is not None or task.get('y') is not None:
                xy_cmd = "G90 G1"
                if task.get('x') is not None: xy_cmd += f" X{task['x']}"
                if task.get('y') is not None: xy_cmd += f" Y{task['y']}"
                xy_cmd += f" F{FEEDRATE_XY}"
                send_gcode(s_grbl, xy_cmd)

            if task.get('z') is not None:
                send_gcode(s_grbl, f"G91 G1 Z{task['z']} F{FEEDRATE_Z}")

            if task.get('grip') is not None:
                send_gripper_cmd(s_grip, 104, task['grip'])
                
            if task.get('pip_z') is not None:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])
                
            if task.get('pipette') is not None:
                send_pipette_oem(s_pip, P_ADDR, task['pipette'])
                
            if task.get('delay', 0) > 0:
                print(f"   └ ⏳ {task['delay']}초 공정 휴식 중...")
                time.sleep(task['delay'])
                
        print("\n==============================================")
        print("✅ 🎉 PASCAL 자동화 공정 완벽 완주 대성공!!!")
        print("==============================================")
        
    except Exception as e:
        print(f"\n❌ 시스템 오류 발생: {e}")
    finally:
        if 's_grbl' in locals() and s_grbl.is_open:
            s_grbl.write(b'$21=0\n') 
            s_grbl.close()
        if 's_grip' in locals() and s_grip.is_open: s_grip.close()
        if 's_pip' in locals() and s_pip.is_open: s_pip.close()
        print("🔌 모든 포트 연결이 안전하게 종료되었습니다.")

if __name__ == "__main__":
    main()