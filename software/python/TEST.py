import serial
import time

# ==========================================
# 1. 시스템 설정
# ==========================================
GRBL_PORT = 'COM3'      # CNC 쉴드 포트
GRIPPER_PORT = 'COM6'   # 그리퍼 포트
BAUD_RATE = 115200

# 이동 목표 (단위: mm)
DIST_X = 400.0   # 40cm (모터 쪽으로)
DIST_Y = -80.0   # 8cm (모터 반대쪽으로)
DIST_Z = 100.0   # 10cm (아래쪽으로)

# 이동 속도 (mm/min)
FEEDRATE_X = 16000  
FEEDRATE_Y = 4000 
FEEDRATE_Z = 2000 

# 그리퍼 설정 (35mm 모델 기준)
GRIP_OPEN = 1000 
GRIP_TARGET = 688

# ==========================================
# 2. 통신 및 제어 함수 정의
# ==========================================

def send_gcode(ser, gcode):
    """G-code 전송 (대기 기능 제거, 순수하게 명령만 전달)"""
    ser.write((gcode + '\n').encode())
    while True:
        # 수정됨: 노이즈 무시 옵션 (errors='ignore') 추가
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'ok' in line:
            print(f"[GRBL] 명령 수신: {gcode}")
            break

def wait_until_idle(ser):
    """기계가 이동을 완전히 멈출 때까지 실시간으로 체크 (딜레이 해결판)"""
    print("...이동 중...", end="", flush=True)
    
    # 1. G-code 명령이 기계에 전달되어 'Run' 상태로 바뀔 수 있게 아주 잠깐 대기
    time.sleep(0.1) 
    
    # 2. 이전에 쌓여있던 쓸데없는 통신 찌꺼기(ok 등)를 완전히 비워서 최신 상태 유지
    ser.reset_input_buffer() 
    
    while True:
        # 3. 핵심 수정: 엔터키(\n) 없이 순수하게 '?' 기호만 전송!
        ser.write(b'?') 
        
        # 4. 상태 읽어오기
        status = ser.readline().decode('utf-8', errors='ignore').strip()
        
        # 상태 메시지(<Run...> 또는 <Idle...>)가 맞는지 확인
        if '<' in status:
            if 'Idle' in status:
                print(" 도착 완료!")
                break
        
        # 너무 빠르게 물어봐서 아두이노가 뻗지 않도록 미세한 휴식
        time.sleep(0.05)

def crc16(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')

def send_gripper_cmd(ser, command, value=0):
    SLAVE_ID = 1
    frame = bytearray([SLAVE_ID, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04])
    frame.append((command >> 8) & 0xFF)
    frame.append(command & 0xFF)
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)
    frame += crc16(frame)
    ser.write(frame)
    print(f"[그리퍼] 명령: {command}, 값: {value}")

# ==========================================
# 3. 메인 시퀀스 실행
# ==========================================

def main():
    try:
        s_grbl = serial.Serial(GRBL_PORT, BAUD_RATE, timeout=2)
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_RATE, timeout=1)
        print("--- PASCAL 통합 공정 시작 ---")
        time.sleep(2) # 안정화

        # ====================================
        # 🚨 [복구된 코드] 그리퍼 초기화 단계
        # ====================================
        print("\n[STEP 0] 그리퍼 초기화 (원점 탐색)...")
        send_gripper_cmd(s_grip, 101, 0)
        time.sleep(5) # 그리퍼가 징~ 하고 원점을 찾는 동안 충분히 대기
        # ====================================

        # [단계 1] X축 이동
        print(f"\n[STEP 1] X축 이동 ({DIST_X}mm)...")
        send_gcode(s_grbl, f"G91 G1 X{DIST_X} F{FEEDRATE_X}")
        wait_until_idle(s_grbl) 
        print("X 이동 완료. 정확히 1초 휴식")
        time.sleep(1)

        # [단계 2] Y축 이동
        print(f"\n[STEP 2] Y축 이동 ({DIST_Y}mm)...")
        send_gcode(s_grbl, f"G91 G1 Y{DIST_Y} F{FEEDRATE_Y}")
        wait_until_idle(s_grbl)
        print("Y 이동 완료. 정확히 1초 휴식")
        time.sleep(1)

        # [단계 3] Z축 이동
        print(f"\n[STEP 3] Z축 이동 ({DIST_Z}mm)...")
        send_gcode(s_grbl, f"G91 G1 Z{DIST_Z} F{FEEDRATE_Z}")
        wait_until_idle(s_grbl)
        print("Z 이동 완료. 정확히 1초 휴식")
        time.sleep(1)

        # [단계 4] 그리퍼 개방 후 파지
        print("\n[STEP 4] 그리퍼 완전 개방...")
        send_gripper_cmd(s_grip, 104, GRIP_OPEN)
        time.sleep(3) 

        print(f"[STEP 5] 25mm 유리 기판 파지 (값: {GRIP_TARGET})...")
        send_gripper_cmd(s_grip, 104, GRIP_TARGET)
        time.sleep(2)

        print("\n--- 모든 공정이 완료되었습니다 ---")

    except Exception as e:
        print(f"오류: {e}")
    finally:
        if 's_grbl' in locals(): s_grbl.close()
        if 's_grip' in locals(): s_grip.close()
        print("포트 연결 종료.")

if __name__ == "__main__":
    main()