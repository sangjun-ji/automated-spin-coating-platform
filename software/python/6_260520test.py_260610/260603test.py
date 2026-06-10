import serial
import time
import sys
import threading
import os

# ==========================================
# [핵심 함수] 핫플레이트 제어용 패킷 생성 (반드시 최상단에 위치)
# ==========================================
def calculate_checksum(packet_str):
    """체크섬 계산 함수"""
    checksum_val = sum(ord(char) for char in packet_str)
    return f"{checksum_val & 0xFF:02X}"

def set_temperature(target_temp, address="01"):
    """목표 온도 설정 패킷 생성 (DWR 명령어)"""
    # 40.0도를 x10 하여 400으로 만든 뒤 16진수로 변환 (400 -> 0190)
    temp_hex = f"{int(target_temp * 10):04X}"
    
    # 0301번지(SV1)에 값 1개를 쓴다(DWR)
    command_body = f"{address}DWR,01,0301,{temp_hex}"
    checksum = calculate_checksum(command_body)
    
    # STX(0x02)와 CR LF(0x0D 0x0A) 조립
    return f"\x02{command_body}{checksum}\r\n"

# ==========================================
# 1. 시스템 통신 및 포트 설정 (★ 장치 포트 확인 필수!)
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

FEEDRATE_XY = 6000     
FEEDRATE_Z = 2000      
SEEK_SPEED_Z = 1000      
SEEK_SPEED_XY = 3000    
PULL_OFF = 5           

P_ADDR = 1        
P_Z_ADDR = 41     

safety_monitoring_active = False  
shutdown_triggered = False

# ==========================================
# 2. 프로세스 레시피
# ==========================================
PROCESS_RECIPE = [
    # [STEP 1] 초기화 및 가열
    {"step": "초기화 확인 및 펌프 대기", "x": None, "y": None, "z": None, "grip": 1000, "pip_z": None, "pipette": "It16000,100,2", "hotplate": None, "pump": 0, "valve": 0, "spin": None, "delay": 1.0},
    {"step": "핫플레이트 30도 가열 개시", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": None, "hotplate": 30.0, "pump": 0, "valve": 0, "spin": None, "delay": 5.0},

    # [STEP 2] 이동 및 흡착 공정
    {"step": "이동", "x": -81.6, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y": -129.8, "z": None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":216, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z": None, "grip": 669, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":100, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  -64.3, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": -336.6, "y":  None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":166.5, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z": None, "grip": 1000, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y":  None, "z":100, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": None, "spin": None, "delay": 0.1},
     # [STEP 3] 코팅
     {"step": "이동", "x": None, "y":  None, "z":100, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": 1, "valve": None, "spin": None, "delay": 3},
     {"step": "이동", "x": None, "y":  None, "z":100, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "valve": 1, "spin": None, "delay": 3},


]

   

    
    # [STEP 3] 코팅 전 준비
    # [STEP 3] 코팅 전 준비
    # [STEP 3] 코팅 전 준비
    # [STEP 4] SAM 도포 # ➔ 💡 여기서 None을 주면 기계가 안 멈추고 영원히 돎! 반드시 0을 줘서 죽여야 함!
    # [STEP 5] Perovskite + antisolvent
    # [STEP 6] 페로브스카이트 박막 결정을 위해 가열된 핫플레이트 위로 최종 안착


# ==========================================
# 3. 제어 및 안전 감시 함수 정의
# ==========================================
def safety_monitor_thread(s_safe, s_grbl):
    global safety_monitoring_active, shutdown_triggered
    print("🛡️ [안전 시스템] 반대편 리미트 센서 실시간 감시 쓰레드 가동")
    
    while True:
        if shutdown_triggered: 
            break
            
        if SAFETY_PORT == 'DISABLE':
            time.sleep(1.0)
            continue

        try:
            if s_safe.in_waiting > 0:
                raw_bytes = s_safe.read(s_safe.in_waiting)
                raw_data = raw_bytes.decode('utf-8', errors='ignore').upper()
                
                if raw_data.strip():
                    print(f"📡 [안전 센서 수신 신호]: {raw_data.strip()}")
                
                if safety_monitoring_active:
                    if any(k in raw_data for k in ['EMERGENCY', 'LIMIT', 'HIT', 'X_LIMIT', 'Y_LIMIT', 'Z_LIMIT']):
                        shutdown_triggered = True
                        print(f"\n🚨🚨🚨 [비상 정지 트리거] 안전 센서 반응 확인!!")
                        try:
                            s_grbl.write(b'!\n')
                            time.sleep(0.01)
                            s_grbl.write(b'\x18')
                            s_grbl.flush()
                            s_grbl.close() 
                        except: pass
                        print("🔌 기계 파손 방지를 위해 OS 레벨에서 셧다운합니다.")
                        os._exit(1) 
        except Exception as e:
            pass
        
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
        time.sleep(0.5) 
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
            
            if line: 
                print(f"      💬 [GRBL 응답]: {line}")
            
            if 'ok' in line: 
                break  # 명령 접수 확인! 하지만 여기서 바로 끝나면 안 됩니다.
            if 'error' in line.lower() or 'ALARM' in line.upper():
                print(f"      🚨 GRBL이 이동을 거부했습니다! 무한 대기 탈출!")
                return 
                
        # ★ [추가된 핵심 코드] GRBL 상태가 'Idle(대기)'로 바뀔 때까지 파이썬을 여기서 멈춰놓고 기다립니다.
        wait_until_idle(ser)
        
        print(f"   ✅ [GRBL] 목적지 도착 완료")
    except Exception as e:
        print(f"❌ 이동 에러: {e}")
        os._exit(1)

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
    print("   ✅ [Z축 상단 센서 확인 완료]"); time.sleep(0.1)

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
    # 💡 모든 리셋(\x18)이 끝난 후, X/Y/Z 0점을 뇌에 한꺼번에 쐐기 박기!
    s.write(b'G92 X0 Y0 Z0\n'); print("   ✅ [X, Y, Z축 0점 세팅 최종 완료!]"); time.sleep(0.5)
    print("\n🔒 [이중 안전] GRBL 하드웨어 및 소프트 리미트 2차 보험 설정 주입 중...")
    s.write(b'$130=630.000\n'); time.sleep(0.1); s.write(b'$131=180.000\n'); time.sleep(0.1); s.write(b'$132=210.000\n'); time.sleep(0.1); s.write(b'$20=0\n'); time.sleep(0.1)        
    s.write(b'$21=1\n'); time.sleep(0.1)
    print("   ✅ GRBL 물리 센서 및 소프트 한계 가두기 적용 완료")

# ==========================================
# 4. 메인 제어 루프
# ==========================================
def main():
    global safety_monitoring_active, shutdown_triggered
    try:
        print("--- 하드웨어 포트 단계별 순차 연결 개시 ---")
        s_grbl = serial.Serial(GRBL_PORT, BAUD_GRBL, timeout=1)
        time.sleep(0.5)
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        time.sleep(1.5)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        time.sleep(0.5)
        
        # 핫플레이트 포트 정상 연결
        s_hotplate = serial.Serial(HOTPLATE_PORT, baudrate=9600, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=2)
        time.sleep(1.0)

        s_safe = None
        if SAFETY_PORT != 'DISABLE':
            s_safe = serial.Serial(SAFETY_PORT, BAUD_SAFE, timeout=1)
        time.sleep(0.5)
        
        s_spin = serial.Serial(SPIN_PORT, BAUD_SPIN, timeout=1)
        time.sleep(3.0) 

        print("\n🔒 [초기 세이프티 가드] 스핀코터 모터 강제 오프 및 진공 흡착 제어 신호 소거")
        s_spin.write(b"0\n")
        s_spin.flush()
        time.sleep(0.5)

        s_grbl.write(b"$X\n")
        s_grbl.flush()
        time.sleep(0.5)

        t = threading.Thread(target=safety_monitor_thread, args=(s_safe, s_grbl), daemon=True)
        t.start()

        print("\n🛡️ [위험 방지] 피펫 Z축 최상단 안전 위치로 사전 대피 시동...")
        send_pipette_oem(s_pip, P_Z_ADDR, "Zz50000")
        time.sleep(3.0) 

        # 호밍 전 잠금 해제
        s_grbl.write(b"$X\n")
        time.sleep(0.5)
        
        run_xyz_homing_hybrid(s_grbl)

        safety_monitoring_active = True
        print("\n🟢 [안전 커튼 전격 활성화] 이제부터 이동 중 어떤 센서든 자극되면 즉시 셧다운됩니다.")

        print("\n[INIT] 그리퍼 초기화 (원점 탐색)...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(4.0) 

        print("\n--- PASCAL 통합 공정 레시피 시작 ---")
        for idx, task in enumerate(PROCESS_RECIPE):
            if shutdown_triggered: os._exit(1)
            print(f"\n▶ [STEP {idx+1}] {task['step']}")
            
            # ★ [가장 중요한 핵심 픽스] 모든 동작(이동) 전 GRBL의 잠재적 알람(Soft Limit 락) 강제 해제
            s_grbl.write(b"$X\n")
            time.sleep(0.05)
            s_grbl.reset_input_buffer()
            
            if task.get('x') is not None or task.get('y') is not None:
                xy_cmd = "G90 G1"
                if task.get('x') is not None: xy_cmd += f" X{task['x']}"
                if task.get('y') is not None: xy_cmd += f" Y{task['y']}"
                xy_cmd += f" F{FEEDRATE_XY}"
                send_gcode(s_grbl, xy_cmd)

            if task.get('z') is not None:
                s_grbl.write(b"$X\n")
                time.sleep(0.05)
                # 💡 [핵심 픽스] $X로 풀린 절대좌표(G90) 모드를 확실하게 다시 주입
                send_gcode(s_grbl, f"G90 G1 Z{task['z']} F{FEEDRATE_Z}")
                
            if task.get('grip') is not None:
                send_gripper_cmd(s_grip, 104, task['grip'])
                
            if task.get('pip_z') is not None:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])
                
            if task.get('pipette') is not None:
                send_pipette_oem(s_pip, P_ADDR, task['pipette'])
                
            # 핫플레이트 설정 로직 적용 완료
            if task.get('hotplate') is not None:
                packet = set_temperature(task['hotplate']) 
                s_hotplate.write(packet.encode('ascii'))
                print(f"   └ [핫플레이트] {task['hotplate']}도 설정 명령 전송 완료!")
            # 💡 [수정] 진공 펌프(P)와 솔레노이드 밸브(V) 개별 제어
            
            # 1. 진공 펌프 제어
            if task.get('pump') is not None:
                pump_cmd = f"P{task['pump']}\n"
                s_spin.write(pump_cmd.encode('ascii'))
                state_str = "ON" if task['pump'] == 1 else "OFF"
                print(f"   └ [진공 펌프] {state_str} 전송 완료!")
                time.sleep(0.1)

            # 2. 솔레노이드 밸브 제어
            if task.get('valve') is not None:
                valve_cmd = f"V{task['valve']}\n"
                s_spin.write(valve_cmd.encode('ascii'))
                state_str = "OPEN" if task['valve'] == 1 else "CLOSE"
                print(f"   └ [솔레노이드 밸브] {state_str} 전송 완료!")
                time.sleep(0.1)    
     

                # 💡 [새로 추가 스핀제어
            if task.get('spin') is not None:
                # 💡 'S'(PWM)가 아닌 'R'(Target RPM)로 전송!
                spin_cmd = f"R{task['spin']}\n" 
                s_spin.write(spin_cmd.encode('ascii'))
                print(f"   └ [스핀코터] 목표 RPM {task['spin']} 자동 PID 제어 명령 전송!")
                time.sleep(0.1)

            # ★ 딜레이는 무조건 해당 스텝의 맨 마지막에 실행되어야 합니다!
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
                s_grbl.write(b'$21=1\n') 
                s_grbl.close()                
            if 's_hotplate' in locals() and s_hotplate.is_open:
                s_hotplate.close()                
            if 's_grip' in locals() and s_grip.is_open: s_grip.close()
            if 's_pip' in locals() and s_pip.is_open: s_pip.close()
            if 's_safe' in locals() and s_safe is not None and s_safe.is_open: s_safe.close()
            if 's_spin' in locals() and s_spin.is_open: s_spin.close()
        except: pass
        print("🔌 모든 포트 연결이 안전하게 종료되었습니다.")

if __name__ == "__main__":
    main()