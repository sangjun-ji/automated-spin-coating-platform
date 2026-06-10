import serial
import time

# ==========================================
# 테스트 설정
# ==========================================
GRIPPER_PORT = 'COM6'   # 장치 관리자에서 USB Serial Port가 COM6가 맞는지 다시 확인하세요!
TEST_BAUD_RATES = [115200, 9600] # 공장 초기값일 수 있는 두 가지 속도 테스트
SLAVE_ID = 1

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

def send_and_read(ser, command, value):
    """명령을 보내고 그리퍼의 대답을 들어보는 함수"""
    frame = bytearray([SLAVE_ID, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04])
    frame.append((command >> 8) & 0xFF)
    frame.append(command & 0xFF)
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)
    frame += crc16(frame)
    
    ser.write(frame)
    time.sleep(0.1) # 대답을 기다림
    
    # 그리퍼가 보낸 대답 읽기
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        hex_resp = " ".join([f"{x:02X}" for x in response])
        return hex_resp
    else:
        return None

def main():
    print("--- 그리퍼 통신 진단 시작 ---\n")
    
    for baud in TEST_BAUD_RATES:
        print(f"[{baud}] 보드레이트로 연결 시도 중...")
        try:
            s_grip = serial.Serial(GRIPPER_PORT, baud, timeout=1)
            time.sleep(1)
            
            # 101: 그리퍼 초기화 명령 테스트
            print(">> 초기화(101) 명령 전송 중...")
            reply = send_and_read(s_grip, 101, 0)
            
            if reply:
                print(f"✅ 통신 성공! 그리퍼 응답 데이터: {reply}")
                print(f"-> 축하합니다. 현재 그리퍼의 속도는 {baud}입니다.")
                s_grip.close()
                return # 성공했으므로 종료
            else:
                print("❌ 응답 없음 (무시됨)")
            
            s_grip.close()
        except serial.SerialException:
            print(f"⚠️ {GRIPPER_PORT} 포트를 열 수 없습니다. 포트 번호가 맞는지, 다른 프로그램이 쓰고 있는지 확인하세요.")
            return
            
    print("\n[진단 결과] 그리퍼가 어떤 속도에서도 대답하지 않습니다.")
    print("1. RS-485 A선과 B선을 서로 바꿔서 꽂아보세요.")
    print("2. 24V 전원이 그리퍼에 잘 들어가는지 확인하세요.")
    print("3. 장치 관리자에서 COM포트 번호가 바뀌지 않았는지 확인하세요.")

if __name__ == "__main__":
    main()