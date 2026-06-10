import serial
import time

# --- [환경 설정] ---
PORT = 'COM6'
BAUD = 38400
P_ADDR = 1  # SP20 장치 주소 [cite: 505]

try:
    # timeout을 0.5초로 설정하여 응답 대기 시간을 확보합니다.
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    print(f"✅ {PORT} 연결 성공")
except Exception as e:
    print(f"❌ 연결 실패: {e}")
    exit()

def send_pipette_cmd(addr, cmd_str):
    """
    OEM 프로토콜 규격으로 명령 전송 및 응답 확인 [cite: 181, 184]
    """
    # 1. OEM 패킷 생성 [cite: 181]
    header = 0xAA
    cmd_bytes = cmd_str.encode('ascii')
    frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
    # 체크섬 계산 [cite: 181]
    checksum = sum(frame) % 256
    frame.append(checksum)
    
    ser.reset_input_buffer()
    ser.write(frame)
    print(f"\n▶ [{addr}번] 전송: [{cmd_str}]")
    
    time.sleep(0.3)
    res = ser.read(ser.in_waiting or 100)
    
    if res:
        if 0x55 in res:
            idx = res.find(0x55)
            try:
                status = res[idx + 2] # 상태 코드 추출 [cite: 184]
                print(f"📡 응답 수신! (상태 코드: {status})")
                return status
            except IndexError: pass
        print(f"⚠️ 데이터 수신(Raw): {res.hex().upper()} (헤더 미발견)")
    else:
        print("📭 응답 없음")
    return None

# --- [메인 실행 로직] ---
try:
    # 1단계: 피펫 초기화 (It) [cite: 254]
    # n3=2: 수동 장착된 팁 유지 [cite: 283]
    print("\n🚀 [1단계] 피펫 초기화 시작...")
    send_pipette_cmd(P_ADDR, "It16000,100,2")
    
    print("⏳ 피스톤 초기 위치 복귀 대기 (10초)...")
    time.sleep(10)

    # 2단계: 사용자 요청 대기
    print("⏳ 추가 대기 중 (5초)...")
    time.sleep(5)

    # 3단계: 50uL 액체 분출 및 2uL 재흡입 (Da) [cite: 315]
    # n1=5000: 분출 부피 (50uL, K=1 기준) [cite: 324]
    # n2=200 : 재흡입 부피 (2uL) -> 분출 후 방울을 팁 안으로 살짝 당김 
    # n3=200 : 분출 속도 (uL/s) [cite: 328]
    # n4=50  : 정지 속도 (uL/s) -> 높을수록 더 날카롭게 끊김 
    
    print("\n🚀 [2단계] 50uL 분출 및 방울 방지 재흡입 시작...")
    send_pipette_cmd(P_ADDR, "Da5000,500,200,50")
    
    print("\n🎉 모든 명령 전송 및 시퀀스 완료!")

except KeyboardInterrupt:
    print("\n🛑 중단됨")
finally:
    ser.close()
    print("🔌 포트가 안전하게 닫혔습니다.")