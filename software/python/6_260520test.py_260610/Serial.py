import serial
import time

# ==========================================
# 1. 설정 (COM6 / 35mm 스트로크 모델)
# ==========================================
PORT = 'COM6'
BAUDRATE = 115200
SLAVE_ID = 1

# 목표 값 (706 = 약 24.7mm)
# 이 수치를 키우면 더 벌어지고, 줄이면 더 꽉 집습니다.
TARGET_VAL = 706 

# ==========================================
# 2. 통신 함수
# ==========================================
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
    frame = bytearray([SLAVE_ID, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04])
    frame.append((command >> 8) & 0xFF)
    frame.append(command & 0xFF)
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)
    frame += crc16(frame)
    ser.write(frame)
    print(f"[전송] 명령: {command}, 값: {value}")

# ==========================================
# 3. 메인 실행 시퀀스 (유리 파손 방지 버전)
# ==========================================
def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"--- {PORT} 연결 성공 (유리 기판 파지 모드) ---")

        # [단계 1] 파지력 하향 조절 (Safety First)
        # 유리가 깨지지 않도록 토크를 절반(50%)으로 낮춥니다.
        print("\n[STEP 1] 파지 토크 50%로 하향 설정 (안전)")
        send_gripper_cmd(ser, 212, 50) 
        time.sleep(0.5)

        # [단계 2] 먼저 완전히 벌리기 (35mm)
        # 초기화 동작 없이 바로 벌립니다.
        print("\n[STEP 2] 완전히 벌리는 중 (35mm)...")
        send_gripper_cmd(ser, 104, 1000)
        time.sleep(3)

        # [단계 3] 설정한 위치로 서서히 모으기 (약 24.7mm)
        print(f"\n[STEP 3] 목표 지점({TARGET_VAL})으로 이동...")
        send_gripper_cmd(ser, 104, TARGET_VAL)
        time.sleep(2)

        print("\n--- 동작 완료! 유리가 안전하게 잡혔는지 확인하세요. ---")

    except Exception as e:
        print(f"\n[오류] {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("포트 연결 종료")

if __name__ == "__main__":
    main()