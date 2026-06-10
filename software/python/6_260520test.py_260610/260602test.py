import serial
import time
import sys
import threading
import os

# ==========================================
# 1. 시스템 통신 및 포트 설정 (장치 추가 반영)
# ==========================================
GRBL_PORT     = 'COM3'       
GRIPPER_PORT  = 'COM12'   
PIPETTE_PORT  = 'COM13'   
SAFETY_PORT   = 'COM14'    
HOTPLATE_PORT = 'COM15'  # 파스칼 핫플레이트 전용 포트 추가
SPIN_PORT     = 'COM5'  # 아두이노 DIY 스핀코터 전용 포트 추가

BAUD_GRBL = 115200
BAUD_GRIP = 115200      
BAUD_PIP  = 38400
BAUD_SAFE = 115200       
BAUD_HOT  = 9600         # 핫플레이트 고유 보레이트
BAUD_SPIN = 115200       # 아두이노 시리얼 보레이트

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
# 2. 프로세스 레시피 (위 정의 내용 배치됨)
# ==========================================
# [위의 PROCESS_RECIPE 선언 데이터가 적용됩니다.]
PROCESS_RECIPE = [
    # [STEP 1] 공정 시작 전 초기화 및 핫플레이트 100도 선행 가열
    {"step": "초기화 확인 및 펌프 대기", "x": None, "y": None, "z": None, "grip": 1000, "pip_z": None, "pipette": "It16000,100,2", "hotplate": None, "spin": None, "delay": 1.0},
    {"step": "핫플레이트 100도 가열 개시", "x": None, "y": None, "z": None, "grip": None, "pip_z": None, "pipette": None, "hotplate": 100.0, "spin": None, "delay": 5.0}, # 100도 도달 대기용 딜레이

    # [STEP 2] 기존 유리판을 스핀코터위로 이동
  
    # [STEP 3] 코팅 전 준비

    # [STEP 4] SAM 도포
  
    # [STEP 5] Perovskite + antisolvent
    

  # [STEP 6] 페로브스카이트 박막 결정을 위해 가열된 핫플레이트 위로 최종 안착
  
]

# ==========================================
# 3. 데이터 패킷 프로토콜 전용 연산 함수
# ==========================================
def crc16(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc.to_bytes(2, 'little')

def calculate_hotplate_checksum(packet_str):
    """핫플레이트 ASCII 패킷 체크섬 계산"""
    checksum_val = sum(ord(char) for char in packet_str)
    return f"{checksum_val & 0xFF:02X}"

def make_hotplate_packet(target_temp, address="01"):
    """핫플레이트 목표 온도 조립 패킷 생성 (DWR)"""
    temp_hex = f"{int(target_temp * 10):04X}"
    command_body = f"{address}DWR,01,0301,{temp_hex}"
    checksum = calculate_hotplate_checksum(command_body)
    return f"\x02{command_body}{checksum}\r\n"

# ==========================================
# 4. 하드웨어별 전용 액션 함수
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
                        os._exit(1) 
            except: pass
        time.sleep(0.005)

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

def control_hotplate(ser, target_temp):
    """핫플레이트 SV 목표 제어값 전송 및 수신 검증"""
    if shutdown_triggered: return
    try:
        ser.reset_input_buffer()
        packet = make_hotplate_packet(target_temp)
        ser.write(packet.encode('ascii'))
        print(f"   └ [핫플레이트] 가열 패킷 송신 ➔ 목표 {target_temp}℃")
        
        response = ser.readline()
        clean_resp = response.decode('ascii', errors='ignore').replace('\x00', '').strip()
        if "OK" in clean_resp:
            print(f"      ✅ [핫플레이트 응답 통신 완벽]: {clean_resp}")
        else:
            print(f"      ⚠️ [핫플레이트 미확인 응답 수신]: {clean_resp}")
    except: os._exit(1)

def control_spin_coater(ser, pwm_value):
    """아두이노 스핀코터 유닛 구동 명령 주입"""
    if shutdown_triggered: return
    try:
        cmd = f"{pwm_value}\n"
        ser.write(cmd.encode())
        print(f"   └ [스핀코터 시리얼] 명령 전송 ➔ PWM: {pwm_value}")
        time.sleep(0.1)
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
            if 'ALARM' in status.upper(): os._exit(1)
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
    time.sleep(0.1); s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=0\n'); time.sleep(0.1)
    s.write(f'G91 G1 Z{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)
    s.write(b'G92 Z0\n'); time.sleep(0.1)

    print("\n▶ [2/3] X축 센서 탐색 중...")
    s.write(b'\x18'); time.sleep(1.0); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=1\n'); time.sleep(0.1); s.reset_input_buffer()
    s.write(f'G91 G1 X2000 F{SEEK_SPEED_XY}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True) 
    time.sleep(0.1); s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=0\n'); time.sleep(0.1)
    s.write(f'G91 G1 X-{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)

    print("\n▶ [3/3] Y축 센서 탐색 중...")
    s.write(b'\x18'); time.sleep(1.0); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=1\n'); time.sleep(0.1); s.reset_input_buffer()
    s.write(f'G91 G1 Y2000 F{SEEK_SPEED_XY}\n'.encode())
    wait_for_hardware_alarm(s, filter_noise=True)
    time.sleep(0.1); s.write(b'\x18'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.1); s.write(b'$21=0\n'); time.sleep(0.1)
    s.write(f'G91 G1 Y-{PULL_OFF} F500\n'.encode()) 
    wait_until_idle(s)
    s.write(b'G92 X0 Y0\n'); time.sleep(0.5)

    s.write(b'$130=630.000\n'); time.sleep(0.1); s.write(b'$131=170.000\n'); time.sleep(0.1); s.write(b'$132=210.000\n'); time.sleep(0.1); s.write(b'$20=1\n'); time.sleep(0.1)        
    s.write(b'$21=1\n'); time.sleep(0.1)
    print("   ✅ GRBL 물리 센서 및 소프트 한계 가두기 적용 완료")

# ==========================================
# 5. 메인 자동화 레시피 인터프리터 루프
# ==========================================
def main():
    global safety_monitoring_active, shutdown_triggered
    try:
        print("--- 하드웨어 포트 일괄 열기 개시 ---")
        s_grbl = serial.Serial(GRBL_PORT, BAUD_GRBL, timeout=1)
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        s_safe = serial.Serial(SAFETY_PORT, BAUD_SAFE, timeout=1) 
        
        # 신규 통신 장치 활성화
        s_hot  = serial.Serial(HOTPLATE_PORT, BAUD_HOT, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=2)
        s_spin = serial.Serial(SPIN_PORT, BAUD_SPIN, timeout=1)
        time.sleep(1.5) # 보드 아두이노 오토 리셋 안정화 대기

        t = threading.Thread(target=safety_monitor_thread, args=(s_safe, s_grbl), daemon=True)
        t.start()

        # [🚨 세이프티] 피펫 Z축 상단 최우선 회피
        print("\n🛡️ [위험 방지] 피펫 Z축 최상단 안전 위치로 사전 대피 시동...")
        send_pipette_oem(s_pip, P_Z_ADDR, "Zz50000")
        time.sleep(3.0) 

        # 메인 갠트리 영점 조절
        run_xyz_homing_hybrid(s_grbl)
        safety_monitoring_active = True
        print("\n🟢 [안전 커튼 활성화 완비]")

        # 그리퍼 원점 조절
        print("\n[INIT] 그리퍼 초기화 (원점 탐색)...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(4.0) 

        print("\n--- 🚀 [PASCAL 5대 모듈 통합 자동화 레시피 엔진 구동] ---")
        for idx, task in enumerate(PROCESS_RECIPE):
            if shutdown_triggered: os._exit(1)
            print(f"\n▶ [STEP {idx+1}] {task['step']}")
            
            # ① 핫플레이트 오븐 온도 설정 조작
            if task.get('hotplate') is not None:
                control_hotplate(s_hot, task['hotplate'])

            # ② 스핀코터 회전 RPM/PWM 변경 조작
            if task.get('spin') is not None:
                control_spin_coater(s_spin, task['spin'])

            # ③ 로봇 XY 절대축 평면 이동 제어
            if task.get('x') is not None or task.get('y') is not None:
                xy_cmd = "G90 G1"
                if task.get('x') is not None: xy_cmd += f" X{task['x']}"
                if task.get('y') is not None: xy_cmd += f" Y{task['y']}"
                xy_cmd += f" F{FEEDRATE_XY}"
                send_gcode(s_grbl, xy_cmd)

            # ④ 로봇 Z 상대축 이송 제어
            if task.get('z') is not None:
                send_gcode(s_grbl, f"G91 G1 Z{task['z']} F{FEEDRATE_Z}")

            # ⑤ 그리퍼 개폐 파지 제어
            if task.get('grip') is not None:
                send_gripper_cmd(s_grip, 104, task['grip'])
                
            # ⑥ 피펫 모듈 고유 Z 승강축 제어
            if task.get('pip_z') is not None:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])
                
            # ⑦ 피펫 펌프 솔루션 디스펜싱 트리거 제어
            if task.get('pipette') is not None:
                send_pipette_oem(s_pip, P_ADDR, task['pipette'])
                
            # ⑧ 공정 휴식 및 물리 지연 시간 타이머 가동
            if task.get('delay', 0) > 0:
                print(f"   └ ⏳ {task['delay']}초간 공정 제어 지연 유지 중...")
                time.sleep(task['delay'])
                
        print("\n==============================================")
        print("✅ 🎉 PASCAL 통합 소자 공정 시스템 완주 대성공!!!")
        print("==============================================")
        
    except Exception as e:
        print(f"\n❌ 연동 제어중 크리티컬 오류 포착: {e}")
    finally:
        shutdown_triggered = True
        safety_monitoring_active = False
        try:
            if 's_grbl' in locals() and s_grbl.is_open:
                s_grbl.write(b'$21=0\n'); s_grbl.close()
            if 's_grip' in locals() and s_grip.is_open: s_grip.close()
            if 's_pip' in locals() and s_pip.is_open: s_pip.close()
            if 's_safe' in locals() and s_safe.is_open: s_safe.close()
            if 's_hot' in locals() and s_hot.is_open: s_hot.close()
            if 's_spin' in locals() and s_spin.is_open: s_spin.close()
        except: pass
        print("🔌 모든 이종 장치 하드웨어 포트가 완전히 리셋 및 폐쇄되었습니다.")

if __name__ == "__main__":
    main()