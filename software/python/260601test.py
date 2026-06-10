import serial
import time
import sys
import threading
import os

# ==========================================
# 1. 시스템 통신 및 포트 설정
# ==========================================
GRBL_PORT = 'COM3'       
GRIPPER_PORT = 'COM12'   
PIPETTE_PORT = 'COM13'   
SAFETY_PORT = 'COM14'    

BAUD_GRBL = 115200
BAUD_GRIP = 115200      
BAUD_PIP  = 38400
BAUD_SAFE = 115200       

FEEDRATE_XY = 20000     
FEEDRATE_Z = 4000      
SEEK_SPEED_Z = 1000      
SEEK_SPEED_XY = 3000    
PULL_OFF = 5           

P_ADDR = 1        
P_Z_ADDR = 41     

safety_monitoring_active = False  
shutdown_triggered = False

# ==========================================
# 2. 프로세스 레시피 (💡 1단계의 피펫 중복 구동 명령은 제거하여 간소화)
# ==========================================
PROCESS_RECIPE = [
    {"step": "초기화 확인 및 펌프 대기", "x": None, "y": None, "z": None, "grip": 1000, "pip_z": None, "pipette": "It16000,100,2", "delay": 1.0 },
    {"step": "이동", "x": None, "y": None, "z": None, "grip": None,  "pip_z": "Zd37000,50000", "pipette": None,  "delay": 3 },
    {"step": "이동", "x": -81.6, "y": None, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.1 },
    {"step": "이동", "x": None, "y": -129.8, "z": None, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.1 },
    {"step": "이동", "x": None, "y": None, "z": 216, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.1 }, 
    {"step": "25mm 유리 기판 파지", "x": None, "y": None, "z": None, "grip": 669, "pip_z": None, "pipette": None, "delay": 2.0},
    {"step": "이동", "x": None, "y": None, "z": -80, "grip": None,  "pip_z": None, "pipette": None,  "delay": 0.1 }, 
]

# ==========================================
# 3. 제어 및 안전 감시 함수 정의 (기존과 동일)
# ==========================================
def safety_monitor_thread(s_safe, s_grbl):
    global safety_monitoring_active, shutdown_triggered
    print("🛡️ [안전 시스템] COM14 반대편 리미트 센서 실시간 감시 쓰레드 가동")
    
    while True:
        if shutdown_triggered: break
            
        if s_safe.in_waiting > 0:
            try:
                raw_bytes = s_safe.read(s_safe.in_waiting)
                raw_data = raw_bytes.decode('utf-8', errors='ignore').upper()
                
                if raw_data.strip():
                    print(f"📡 [COM14 센서 수신 신호]: {raw_data.strip()}")
                
                if safety_monitoring_active:
                    if any(k in raw_data for k in ['EMERGENCY', 'LIMIT', 'HIT', 'X_LIMIT', 'Y_LIMIT', 'Z_LIMIT']):
                        shutdown_triggered = True
                        print(f"\n🚨🚨🚨 [비상 정지 트리거] COM14 안전 센서 반응 확인!!")
                        try:
                            s_grbl.write(b'!')
                            time.sleep(0.01)
                            s_grbl.write(b'\x18')
                            s_grbl.flush()
                            s_grbl.close() 
                        except: pass
                        print("🔌 기계 파손 방지를 위해 OS 레벨에서 셧다운합니다.")
                        os._exit(1) 
            except Exception as e:
                pass
        time.sleep(0.005)

def crc16(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc.to_bytes(2, 'little')

def send_gripper_cmd(ser, command, value=0):
    if shutdown_triggered: return
    try:
        SLAVE_ID = 1
        frame = bytearray([SLAVE_ID, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04])
        frame += bytearray([(command >> 8) & 0xFF, command & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
        frame += crc16(frame)
        ser.write(frame)
        print(f"   └ [그리퍼] 명령: {command}, value: {value}")
        time.sleep(0.1)
    except: os._exit(1)

def send_pipette_oem(ser, addr, cmd_str):
    if shutdown_triggered: return
    try:
        header = 0xAA
        cmd_bytes = cmd_str.encode('ascii')
        frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
        frame.append(sum(frame) % 256) 
        ser.reset_input_buffer()
        ser.write(frame)
        print(f"   └ [RS485-OEM] 주소 {addr}: {cmd_str}")
        time.sleep(0.2)
    except: os._exit(1)

def wait_for_hardware_alarm(ser, filter_noise=False):
    if filter_noise:
        time.sleep(0.1) 
        try: ser.reset_input_buffer()
        except: os._exit(1)

    timeout_start = time.time()
    while True:
        if shutdown_triggered: os._exit(1)
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if any(keyword in line for keyword in ['ALARM', 'Alarm', 'Reset', 'error:', 'State:Alarm']):
                    return True
        except: os._exit(1)
                
        if time.time() - timeout_start > 30.0: 
            print("\n🚨 [위험] 물리 충돌 신호 포착 실패! 즉시 안전 종료합니다.")
            os._exit(1)
        time.sleep(0.005)

def wait_until_idle(ser):
    time.sleep(0.1) 
    while True:
        if shutdown_triggered: os._exit(1) 
        try:
            ser.write(b'?')
            status = ser.readline().decode('utf-8', errors='ignore').strip()
            if 'Idle' in status: break
            if 'ALARM' in status.upper():
                print("\n🚨🚨🚨 [GRBL 하드웨어 비상] 기존 3축 원점 센서 중 하나가 타격되었습니다! 장비를 셧다운합니다.")
                os._exit(1)
        except: os._exit(1)
        time.sleep(0.05) 

def send_gcode(ser, gcode):
    if shutdown_triggered: return
    try:
        ser.reset_input_buffer()
        ser.write((gcode + '\n').encode())
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if 'ok' in line: break
        print(f"   └ [GRBL] 이동 시작 ➔ {gcode.strip()}")
        wait_until_idle(ser)
        print(f"   ✅ [GRBL] 목적지 도달 완료")
    except: os._exit(1)

def run_xyz_homing_hybrid(s):
    print("--- [PASCAL] 하드웨어 충돌 감지형 3축 통합 호밍 시작 ---")
    s.reset_input_buffer()
    s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=1\n'); time.sleep(0.1); s.reset_input_buffer()

    print("\n▶ [1/3] Z축 상단 센서 탐색 중...")
    s.write(f'G91 G1 Z-2000 F{SEEK_SPEED_Z}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True) 
    print("💥 Z축 물리적 센서 충돌 감지 완료!")
    time.sleep(0.1); s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=0\n'); time.sleep(0.1)
    print(f"   🏃 Z축 {PULL_OFF}mm 하강 후퇴...")
    s.write(f'G91 G1 Z{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)
    s.write(b'G92 Z0\n'); print("   ✅ [Z축 상단 0점 세팅 완료]"); time.sleep(0.1)

    print("\n▶ [2/3] X축 센서 탐색 중...")
    s.write(b'\x18'); time.sleep(1.0); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=1\n'); time.sleep(0.1); s.reset_input_buffer()
    s.write(f'G91 G1 X2000 F{SEEK_SPEED_XY}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True) 
    print("💥 X축 물리적 센서 충돌 감지 완료!")
    time.sleep(0.1); s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=0\n'); time.sleep(0.1)
    print(f"   🏃 X축 {PULL_OFF}mm 후퇴...")
    s.write(f'G91 G1 X-{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)

    print("\n▶ [3/3] Y축 센서 탐색 중...")
    s.write(b'\x18'); time.sleep(1.0); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=1\n'); time.sleep(0.1); s.reset_input_buffer()
    s.write(f'G91 G1 Y2000 F{SEEK_SPEED_XY}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True)
    print("💥 Y축 물리적 센서 충돌 감지 완료!")
    time.sleep(0.1); s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=0\n'); time.sleep(0.1)
    print(f"   🏃 Y축 {PULL_OFF}mm 후퇴...")
    s.write(f'G91 G1 Y-{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)
    s.write(b'G92 X0 Y0\n'); print("   ✅ [X축, Y축 0점 세팅 최종 완료!]"); time.sleep(0.5)

    print("\n🔒 [이중 안전] GRBL 하드웨어 및 소프트 리미트 2차 보험 설정 주입 중...")
    s.write(b'$130=630.000\n'); time.sleep(0.1); s.write(b'$131=170.000\n'); time.sleep(0.1); s.write(b'$132=210.000\n'); time.sleep(0.1); s.write(b'$20=1\n'); time.sleep(0.1)        
    s.write(b'$21=1\n'); time.sleep(0.1)
    print("   ✅ GRBL 물리 센서 및 소프트 한계 가두기 적용 완료")

# ==========================================
# 4. 메인 루프 (💡 피펫 최우선 세이프티 구동 레이아웃 개편)
# ==========================================
def main():
    global safety_monitoring_active, shutdown_triggered
    try:
        print("--- 통신 포트 연결 중 ---")
        s_grbl = serial.Serial(GRBL_PORT, BAUD_GRBL, timeout=1)
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        s_safe = serial.Serial(SAFETY_PORT, BAUD_SAFE, timeout=1) 
        time.sleep(1.0) # 233번 줄 언저리: 보드 안정화 1초 사수

        t = threading.Thread(target=safety_monitor_thread, args=(s_safe, s_grbl), daemon=True)
        t.start()

        # 🚀 [🚨 최우선 순위 조치 - 239번 줄 부근] 
        # 메인 로봇이 0점 잡으려고 요동치기 전에 피펫 Z축부터 안전 구역(최상단)으로 숨깁니다.
        print("\n🛡️ [위험 방지] 피펫 Z축 최상단 안전 위치로 사전 대피 시동...")
        send_pipette_oem(s_pip, P_Z_ADDR, "Zz50000")
        time.sleep(3.0) # 피펫이 완전히 위로 올라갈 수 있도록 물리적 대기 시간 부여

        # 2. 피펫이 안전하게 숨은 뒤 메인 3축 호밍 가동 (245번 줄 부근)
        run_xyz_homing_hybrid(s_grbl)

        safety_monitoring_active = True
        print("\n🟢 [안전 커튼 전격 활성화] 이제부터 이동 중 어떤 센서든 자극되면 즉시 셧다운됩니다.")

        # 3. 그리퍼 초기화 (251번 줄 부근)
        print("\n[INIT] 그리퍼 초기화 (원점 탐색)...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(4.0) # 그리퍼 물리 이동 시간 사수

        print("\n--- PASCAL 통합 공정 레시피 시작 ---")
        for idx, task in enumerate(PROCESS_RECIPE):
            if shutdown_triggered: os._exit(1)
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
        
    except Exception as e:
        print(f"\n❌ 시스템 오류 발생: {e}")
    finally:
        shutdown_triggered = True
        safety_monitoring_active = False
        try:
            if 's_grbl' in locals() and s_grbl.is_open:
                s_grbl.write(b'$21=0\n') 
                s_grbl.close()
            if 's_grip' in locals() and s_grip.is_open: s_grip.close()
            if 's_pip' in locals() and s_pip.is_open: s_pip.close()
            if 's_safe' in locals() and s_safe.is_open: s_safe.close()
        except: pass
        print("🔌 모든 포트 연결이 안전하게 종료되었습니다.")

if __name__ == "__main__":
    main()