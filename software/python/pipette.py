import serial
import time

# --- [환경 설정] ---
PORT = 'COM6'
BAUD = 38400
Z_ADDR = 41  # Z축 장치 주소

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"✅ {PORT} 연결 성공 (노이즈 무시 모드)")
except Exception as e:
    print(f"❌ 포트 연결 실패: {e}")
    exit()

def send_oem(addr, cmd_str):
    """응답을 기다리지 않고 명령만 확실히 전송하는 함수"""
    ser.reset_input_buffer() # 이전의 노이즈 데이터 청소
    
    header = 0xAA
    cmd_bytes = cmd_str.encode('ascii')
    # Frame: Header(1) + Addr(1) + Len(1) + Data(n)
    frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
    # Checksum: 모든 바이트 합의 하위 8비트
    checksum = sum(frame) % 256
    frame.append(checksum)
    
    ser.write(frame)
    print(f"▶ [{addr}번] 명령 전송: [{cmd_str}]")
    
    time.sleep(0.1) 
    ser.reset_input_buffer() 
    return True

# --- [메인 실행 로직] ---
try:
    # 1단계: 원점 초기화 (Zz)
    print("\n🚀 [1단계] Z축 원점 초기화 시작 (Zz20000)")
    send_oem(Z_ADDR, "Zz20000") # 상단 0점 위치를 찾음 [cite: 252, 254]
    
    wait_time = 10 
    print(f"⏳ 초기화 대기 중... ({wait_time}초)")
    time.sleep(wait_time)
    
    # 2단계: 팁 랙 근처까지 하강 (Zd)
    # 현재 위치(원점)에서 아래로 169,000um (169mm) 상대 이동 [cite: 273, 275]
    print("\n🚀 [2단계] 팁 랙 근처로 하강 (Zd135000,50000)")
    send_oem(Z_ADDR, "Zd135000,50000")
    
    print("⏳ 이동 중... (5초)")
    time.sleep(5)

    # 3단계: 피펫 팁 장착 실행 (Zg)
    # Zg[목표좌표],[파워%] 형식입니다. 
    # 175000: 절대 위치 175mm까지 이동하며 팁을 탐색 (랙 위치에 맞춰 조정 필요) 
    # 80: 모터 파워 80% 사용 (권장 범위 80-100%) [cite: 283, 285]
    print("\n🚀 [3단계] 팁 장착 명령 전송 (Zg5000,80)")
    send_oem(Z_ADDR, "Zg5000,80") # 팁 감지 시 스톨로 인해 자동 정지함 [cite: 279]
    
    print("⏳ 장착 동작 대기 중... (3초)")
    time.sleep(3)

    # 4단계: 5cm 위로 상승 (Zu)
    # 현재 위치에서 위로 50,000um (50mm) 상대 이동 
    print("\n🚀 [4단계] 5cm 위로 상승 (Zu100000,50000)")
    send_oem(Z_ADDR, "Zu100000,50000")
    
    print("⏳ 상승 중... (3초)")
    time.sleep(3)

    print("\n🎉 모든 시퀀스(초기화-하강-장착-상승) 완료!")

except KeyboardInterrupt:
    print("\n🛑 사용자에 의해 중단되었습니다.")
finally:
    ser.close()
    print("🔌 포트가 안전하게 닫혔습니다.")