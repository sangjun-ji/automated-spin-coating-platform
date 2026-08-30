import serial
import time

# --- 설정 (포트 번호를 본인 환경에 맞게 확인하세요) ---
SPIN_PORT = 'COM7'     # 드론 모터용 UNO
SAFETY_PORT = 'COM18'  # 진공 시스템용 MEGA
BAUD_RATE = 115200
TARGET_PWM = 940      # 모터가 안정적으로 도는 값(이전 테스트 결과에 따라 조정)

def soft_start(ser, target, step=2, delay=0.05):
    print(f"📈 [드론] 소프트 스타트: 900 -> {target}")
    for current_pwm in range(900, target + 1, step):
        ser.write(f"R{current_pwm}\n".encode('ascii'))
        time.sleep(delay)

def main():
    print("--- 🚁+🗜️ 드론 & 진공 통합 자동화 테스트 ---")
    
    try:
        # 1. 포트 연결
        s_spin = serial.Serial(SPIN_PORT, BAUD_RATE, timeout=1)
        s_safe = serial.Serial(SAFETY_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("✅ 모든 보드 연결 성공!")

        # 2. ESC 암밍
        print("🔒 [드론] 초기화 신호 전송 (5초 대기)")
        s_spin.write(b"R900\n")
        time.sleep(5)

        # 3. 공정 시작: 펌프 ON -> 밸브 ON -> 모터 회전
        print("\n💨 1. 펌프 ON -> 밸브 ON (진공 흡착)")
        s_safe.write(b"P1\n") 
        time.sleep(0.5)
        s_safe.write(b"V1\n")
        time.sleep(1) # 진공 잡힐 시간

        print("🚀 2. 모터 가동 시작")
        soft_start(s_spin, TARGET_PWM, step=2, delay=0.05)
        
        print("🔄 3. 10초간 회전 유지...")
        time.sleep(10)

        # 4. 공정 종료: 모터 정지 -> 밸브 OFF -> 펌프 OFF
        print("\n🛑 4. 모터 정지 (R900)")
        s_spin.write(b"R900\n")
        time.sleep(5)

        print("🛑 5. 밸브 OFF -> 펌프 OFF (진공 해제)")
        s_safe.write(b"V0\n")
        time.sleep(0.5)
        s_safe.write(b"P0\n")

        print("\n🎉 통합 테스트가 성공적으로 종료되었습니다.")

    except Exception as e:
        print(f"❌ 에러 발생: {e}")
    finally:
        # 안전한 종료
        if 's_spin' in locals() and s_spin.is_open:
            s_spin.write(b"R900\n")
            s_spin.close()
        if 's_safe' in locals() and s_safe.is_open:
            s_safe.write(b"V0\n")
            s_safe.write(b"P0\n")
            s_safe.close()
        print("🔌 모든 포트를 닫았습니다.")

if __name__ == "__main__":
    main()