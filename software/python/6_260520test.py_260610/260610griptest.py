import serial
import time
import sys
import threading
import os
import tkinter as tk  

# ==========================================
# [핵심 함수] 핫플레이트 제어용 패킷 생성
# ==========================================
def calculate_checksum(packet_str):
    checksum_val = sum(ord(char) for char in packet_str)
    return f"{checksum_val & 0xFF:02X}"

def set_temperature(target_temp, address="01"):
    temp_hex = f"{int(target_temp * 10):04X}"
    command_body = f"{address}DWR,01,0301,{temp_hex}"
    checksum = calculate_checksum(command_body)
    return f"\x02{command_body}{checksum}\r\n"

# ==========================================
# 1. 시스템 통신 및 포트 설정
# ==========================================
GRBL_PORT     = 'COM3'       
GRIPPER_PORT  = 'COM12'   
PIPETTE_PORT  = 'COM13'   
SAFETY_PORT   = 'COM14'   
HOTPLATE_PORT = 'COM15'   
SPIN_PORT     = 'COM5'    

BAUD_GRBL = 115200
BAUD_GRIP = 115200      
BAUD_PIP  = 38400
BAUD_SAFE = 115200       
BAUD_SPIN = 115200        

FEEDRATE_XY = 15000     
FEEDRATE_Z = 4000      
SEEK_SPEED_Z = 1000      
SEEK_SPEED_XY = 5000    
PULL_OFF = 5           

P_ADDR = 1        
P_Z_ADDR = 41     

safety_monitoring_active = False  
shutdown_triggered = False

current_spin_rpm = "0"
target_spin_rpm = "0"       
target_hotplate_temp = "대기" 
current_step_name = "장비 초기화 및 연결 대기 중..." 
s_spin_global = None        

# ==========================================
# 2. 프로세스 레시피 (피펫 딜레이 0.1로 통일!)
# ==========================================
PROCESS_RECIPE = [
    {"step": "초기화 확인 및 펌프 대기", "x": None, "y": None, "z": None, "grip": 1000, "pip_z": None, "pipette": "It16000,100,2", "hotplate": None, "pump": 0, "valve": 0, "spin": None, "delay": 0.1},
    
    #셀 이동 to spin coater
    {"step": "이동", "x": -172.4, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y": -62.5, "z": None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":197.2, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z": None, "grip": 669, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":60, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  -69.8, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": -558.9, "y":  None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":128, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z": None, "grip": 1000, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":60, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":128.3, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z": None, "grip": 669, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":60, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":125, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z": None, "grip": 1000, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
   

]
    
# ==========================================
# 3. 제어 및 안전 감시 함수 정의
# ==========================================
def safety_monitor_thread(s_safe, s_grbl):
    global safety_monitoring_active, shutdown_triggered
    print("🛡️ [안전 시스템] 반대편 리미트 센서 실시간 감시 쓰레드 가동")
    
    while True:
        if shutdown_triggered: break
        if SAFETY_PORT == 'DISABLE':
            time.sleep(1.0); continue

        try:
            if s_safe.in_waiting > 0:
                raw_bytes = s_safe.read(s_safe.in_waiting)
                raw_data = raw_bytes.decode('utf-8', errors='ignore').upper()
                if raw_data.strip(): print(f"📡 [안전 센서 수신 신호]: {raw_data.strip()}")
                
                if safety_monitoring_active:
                    if any(k in raw_data for k in ['EMERGENCY', 'LIMIT', 'HIT', 'X_LIMIT', 'Y_LIMIT', 'Z_LIMIT']):
                        shutdown_triggered = True
                        print(f"\n🚨🚨🚨 [비상 정지 트리거] 안전 센서 반응 확인!!")
                        try:
                            s_grbl.write(b'!\n'); time.sleep(0.01); s_grbl.write(b'\x18'); s_grbl.flush(); s_grbl.close() 
                        except: pass
                        print("🔌 기계 파손 방지를 위해 OS 레벨에서 셧다운합니다.")
                        os._exit(1) 
        except: pass
        time.sleep(0.05)

def spin_monitor_thread(s_spin):
    global shutdown_triggered, current_spin_rpm
    while True:
        if shutdown_triggered: break
        try:
            if s_spin.in_waiting > 0:
                line = s_spin.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("RPM:"):
                    current_spin_rpm = line.split(":")[1]
        except: pass
        time.sleep(0.05)

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

# ★ [핵심 업그레이드] 피펫이 동작을 끝낼 때까지 파이썬을 무한 대기시키는 로직
def send_pipette_oem(ser, addr, cmd_str):
    if shutdown_triggered: return
    try:
        header = 0xAA
        cmd_bytes = cmd_str.encode('ascii')
        frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
        checksum = sum(frame) % 256
        frame.append(checksum)
        
        # 다른 쓰레드 간섭 방지를 위해 잠시 인터럽트 차단 느낌으로 전송
        ser.write(frame)
        ser.flush()
        print(f"   └ [RS485-OEM] 전송: {cmd_str}")
        
        # 0606에서 잘 되던 방식: 상태 확인 없이 시간으로 확실하게 제압
        # 복합 명령어일수록 딜레이를 길게 줘야 씹히지 않습니다.
        time.sleep(0.5) 
            
    except Exception as e: 
        print(f"❌ 피펫 에러: {e}")

def wait_for_hardware_alarm(ser, filter_noise=False):
    if filter_noise:
        time.sleep(0.5); ser.reset_input_buffer()
    timeout_start = time.time()
    while True:
        if shutdown_triggered: os._exit(1)
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if any(keyword in line for keyword in ['ALARM', 'Alarm', 'Reset', 'error:', 'State:Alarm']): return True
        except: os._exit(1)
        if time.time() - timeout_start > 30.0: os._exit(1)
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
                print("\n🚨🚨🚨 [GRBL 하드웨어 비상] 3축 원점 센서가 타격되었습니다! 셧다운합니다.")
                os._exit(1)
        except: os._exit(1)
        time.sleep(0.05) 

def send_gcode(ser, gcode):
    if shutdown_triggered: return
    try:
        ser.reset_input_buffer()
        ser.write((gcode + '\n').encode())
        print(f"   └ [GRBL] 명령 전송: {gcode.strip()}")
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line: print(f"      💬 [GRBL 응답]: {line}")
            if 'ok' in line: break 
            if 'error' in line.lower() or 'ALARM' in line.upper(): return 
        wait_until_idle(ser)
        print(f"   ✅ [GRBL] 목적지 도착 완료")
    except Exception as e:
        os._exit(1)

def run_xyz_homing_hybrid(s):
    print("--- [PASCAL] 하드웨어 충돌 감지형 3축 통합 호밍 시작 ---")
    s.reset_input_buffer()
    s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=1\n'); time.sleep(0.1); s.reset_input_buffer()

    print("\n▶ [1/3] Z축 상단 센서 탐색 중...")
    s.write(f'G91 G1 Z-2000 F{SEEK_SPEED_Z}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True) 
    time.sleep(0.1); s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=0\n'); time.sleep(0.1)
    s.write(f'G91 G1 Z{PULL_OFF} F500\n'.encode()); wait_until_idle(s)

    print("\n▶ [2/3] X축 센서 탐색 중...")
    s.write(b'\x18'); time.sleep(1.0); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=1\n'); time.sleep(0.1); s.reset_input_buffer()
    s.write(f'G91 G1 X2000 F{SEEK_SPEED_XY}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True) 
    time.sleep(0.1); s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=0\n'); time.sleep(0.1)
    s.write(f'G91 G1 X-{PULL_OFF} F500\n'.encode()); wait_until_idle(s)

    print("\n▶ [3/3] Y축 센서 탐색 중...")
    s.write(b'\x18'); time.sleep(1.0); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=1\n'); time.sleep(0.1); s.reset_input_buffer()
    s.write(f'G91 G1 Y2000 F{SEEK_SPEED_XY}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True)
    time.sleep(0.1); s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=0\n'); time.sleep(0.1)
    s.write(f'G91 G1 Y-{PULL_OFF} F500\n'.encode()); wait_until_idle(s)
    
    s.write(b'G92 X0 Y0 Z0\n'); print("   ✅ [X, Y, Z축 0점 세팅 최종 완료!]"); time.sleep(0.5)
    s.write(b'$130=630.000\n'); time.sleep(0.1); s.write(b'$131=180.000\n'); time.sleep(0.1); s.write(b'$132=210.000\n'); time.sleep(0.1); s.write(b'$20=0\n'); time.sleep(0.1)        
    s.write(b'$21=1\n'); time.sleep(0.1)

# ==========================================
# 4. 메인 제어 루프
# ==========================================
def automation_task(): 
    global safety_monitoring_active, shutdown_triggered
    global current_step_name, target_spin_rpm, target_hotplate_temp, s_spin_global 
    
    try:
        current_step_name = "포트 연결 및 초기화 진행 중..."
        print("--- 하드웨어 포트 일괄 연결 시작 ---")
        
        s_grbl = serial.Serial(GRBL_PORT, BAUD_GRBL, timeout=1)
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        s_hotplate = serial.Serial(HOTPLATE_PORT, baudrate=9600, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=2)
        
        s_safe = None
        if SAFETY_PORT != 'DISABLE': s_safe = serial.Serial(SAFETY_PORT, BAUD_SAFE, timeout=1)
        s_spin = serial.Serial(SPIN_PORT, BAUD_SPIN, timeout=1)
        s_spin_global = s_spin 
        
        print("⏳ 보드 동시 초기화 대기 중 (2초)...")
        time.sleep(2.0) 

        print("\n🔒 [초기 세이프티 가드] NC 릴레이 강제 OFF 및 스핀코터 대기 모드 진입")
        s_spin.write(b"P0\n"); time.sleep(0.05)
        s_spin.write(b"V0\n"); time.sleep(0.05)
        s_spin.write(b"F0\n"); time.sleep(0.05)
        s_spin.write(b"R0\n"); s_spin.flush()
        
        s_grbl.write(b"$X\n"); s_grbl.flush(); time.sleep(0.5)

        t = threading.Thread(target=safety_monitor_thread, args=(s_safe, s_grbl), daemon=True)
        t.start()

        t_spin = threading.Thread(target=spin_monitor_thread, args=(s_spin,), daemon=True)
        t_spin.start()

        current_step_name = "Z축 회피 및 원점 호밍 시작..."
        print("\n🛡️ [위험 방지] 피펫 Z축 최상단 안전 위치로 사전 대피 시동...")
        send_pipette_oem(s_pip, P_Z_ADDR, "Zz50000"); time.sleep(3.0) 
        s_grbl.write(b"$X\n"); time.sleep(0.5)
        
        run_xyz_homing_hybrid(s_grbl)
        safety_monitoring_active = True
        print("\n🟢 [안전 커튼 전격 활성화] 이제부터 이동 중 어떤 센서든 자극되면 즉시 셧다운됩니다.")

        print("\n[INIT] 그리퍼 초기화 (원점 탐색)...")
        send_gripper_cmd(s_grip, 101, 0); time.sleep(4.0) 

        print("\n--- PASCAL 통합 공정 레시피 시작 ---")
        for idx, task in enumerate(PROCESS_RECIPE):
            if shutdown_triggered: os._exit(1)
            
            current_step_name = f"[STEP {idx+1}] {task['step']}"
            print(f"\n▶ [STEP {idx+1}] {task['step']}")
            
            s_grbl.write(b"$X\n"); time.sleep(0.05); s_grbl.reset_input_buffer()
            
            if task.get('x') is not None or task.get('y') is not None:
                xy_cmd = "G90 G1"
                if task.get('x') is not None: xy_cmd += f" X{task['x']}"
                if task.get('y') is not None: xy_cmd += f" Y{task['y']}"
                xy_cmd += f" F{FEEDRATE_XY}"
                send_gcode(s_grbl, xy_cmd)

            if task.get('z') is not None:
                s_grbl.write(b"$X\n"); time.sleep(0.05)
                send_gcode(s_grbl, f"G90 G1 Z{task['z']} F{FEEDRATE_Z}")
                
            if task.get('grip') is not None:
                send_gripper_cmd(s_grip, 104, task['grip'])
                
            if task.get('pip_z') is not None:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])
                
            if task.get('pipette') is not None:
                send_pipette_oem(s_pip, P_ADDR, task['pipette'])
                
            if task.get('hotplate') is not None:
                target_hotplate_temp = str(task['hotplate'])
                packet = set_temperature(task['hotplate']) 
                s_hotplate.write(packet.encode('ascii'))
                print(f"   └ [핫플레이트] {task['hotplate']}도 설정 명령 전송 완료!")
            
            if task.get('pump') is not None:
                s_spin.write(f"P{task['pump']}\n".encode('ascii')); time.sleep(0.1)
                print(f"   └ [진공 펌프] {'ON' if task['pump'] == 1 else 'OFF'} 전송 완료!")

            if task.get('valve') is not None:
                s_spin.write(f"V{task['valve']}\n".encode('ascii')); time.sleep(0.1)
                print(f"   └ [솔레노이드 밸브] {'OPEN' if task['valve'] == 1 else 'CLOSE'} 전송 완료!")

            if task.get('spin') is not None:
                target_spin_rpm = str(task['spin'])
                s_spin.write(f"R{task['spin']}\n".encode('ascii')); time.sleep(0.1)
                print(f"   └ [스핀코터] 목표 RPM {task['spin']} 자동 PID 제어 명령 전송!")

            if task.get('delay', 0) > 0:
                print(f"   └ ⏳ {task['delay']}초 공정 휴식 중...")
                time.sleep(task['delay'])
                
        current_step_name = "✅ 모든 공정 완주 대성공!"
        print("\n==============================================")
        print("✅ 🎉 PASCAL 자동화 공정 완벽 완주 대성공!!!")

    except Exception as e:
        current_step_name = f"❌ 시스템 오류 발생: {e}"
    finally:
        shutdown_triggered = True
        safety_monitoring_active = False
        try:
            if 's_grbl' in locals() and s_grbl.is_open: s_grbl.write(b'$21=1\n'); s_grbl.close()                
            if 's_hotplate' in locals() and s_hotplate.is_open: s_hotplate.close()                
            if 's_grip' in locals() and s_grip.is_open: s_grip.close()
            if 's_pip' in locals() and s_pip.is_open: s_pip.close()
            if 's_safe' in locals() and s_safe is not None and s_safe.is_open: s_safe.close()
            if 's_spin' in locals() and s_spin.is_open: s_spin.close()
        except: pass

# ==========================================
# 5. GUI 대시보드
# ==========================================
def main():
    root = tk.Tk()
    root.title("PASCAL 통합 대시보드")
    root.geometry("450x450") 
    root.attributes('-topmost', True) 
    root.configure(bg="#f4f4f4")

    frame_status = tk.LabelFrame(root, text="현재 공정 상태", font=("Arial", 10, "bold"), bg="#f4f4f4")
    frame_status.pack(fill="x", padx=15, pady=10)
    lbl_status = tk.Label(frame_status, text="준비 중...", font=("Arial", 13, "bold"), fg="#0033cc", bg="#f4f4f4", wraplength=400)
    lbl_status.pack(pady=10)

    frame_info = tk.LabelFrame(root, text="실시간 파라미터", font=("Arial", 10, "bold"), bg="#f4f4f4")
    frame_info.pack(fill="x", padx=15, pady=5)
    lbl_temp = tk.Label(frame_info, text="🔥 핫플레이트 설정 온도 : 대기", font=("Arial", 12), bg="#f4f4f4")
    lbl_temp.pack(pady=5)
    lbl_rpm_target = tk.Label(frame_info, text="🔄 스핀코터 설정 RPM : 0", font=("Arial", 12), fg="#555555", bg="#f4f4f4")
    lbl_rpm_target.pack()
    lbl_rpm_real = tk.Label(frame_info, text="센서 실제 측정 RPM : 0", font=("Arial", 18, "bold"), fg="#FF3366", bg="#f4f4f4")
    lbl_rpm_real.pack(pady=5)

    frame_manual = tk.LabelFrame(root, text="수동 긴급 제어 (비상 정지용)", font=("Arial", 10, "bold"), fg="red", bg="#f4f4f4")
    frame_manual.pack(fill="x", padx=15, pady=15)

    def manual_pump_off():
        if s_spin_global and s_spin_global.is_open: s_spin_global.write(b"P0\n")
    def manual_valve_off():
        if s_spin_global and s_spin_global.is_open: s_spin_global.write(b"V0\n")
    def manual_fan_off():
        if s_spin_global and s_spin_global.is_open: s_spin_global.write(b"R0\n")

    btn_frame = tk.Frame(frame_manual, bg="#f4f4f4")
    btn_frame.pack(pady=10)
    btn_style = {"font": ("Arial", 10, "bold"), "bg": "#ffb3b3", "width": 12, "height": 2}
    tk.Button(btn_frame, text="펌프 OFF", command=manual_pump_off, **btn_style).pack(side="left", padx=5)
    tk.Button(btn_frame, text="밸브 OFF", command=manual_valve_off, **btn_style).pack(side="left", padx=5)
    tk.Button(btn_frame, text="스핀코터 OFF", command=manual_fan_off, **btn_style).pack(side="left", padx=5)

    def update_gui():
        lbl_status.config(text=current_step_name)
        lbl_temp.config(text=f"🔥 핫플레이트 설정 온도 : {target_hotplate_temp} ℃")
        lbl_rpm_target.config(text=f"🔄 스핀코터 설정 RPM : {target_spin_rpm}")
        lbl_rpm_real.config(text=f"센서 실제 측정 RPM : {current_spin_rpm}")
        
        if shutdown_triggered: root.destroy()
        else: root.after(100, update_gui)

    update_gui()
    t_auto = threading.Thread(target=automation_task, daemon=True)
    t_auto.start()
    root.mainloop()

if __name__ == "__main__":
    main()