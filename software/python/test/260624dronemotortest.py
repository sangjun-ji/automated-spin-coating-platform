import serial
import time

# --- 설정 ---
SPIN_PORT = 'COM7'       # 본인의 UNO 보드 포트에 맞게 수정하세요.
BAUD_SPIN = 115200
TARGET_PWM = 940         # 첫 테스트는 안전하게 950으로 설정했습니다.

def soft_start(ser, target, step=2, delay=0.05):
    """파워서플라이 보호를 위한 부드러운 가속 함수"""
    print(f"📈 소프트 스타트 시작: 900 -> {target} (증가량: {step})")
    for current_pwm in range(900, target + 1, step):
        ser.write(f"R{current_pwm}\n".encode('ascii'))
        time.sleep(delay)
    print("✅ 목표 속도 도달 완료!")

def main():
    print(f"--- 🚁 드론 모터(UNO) 단독 제어 테스트 ---")
    try:
        # 1. 시리얼 연결
        s_spin = serial.Serial(SPIN_PORT, BAUD_SPIN, timeout=1)
        time.sleep(2) # 보드 재부팅 대기
        print(f"✅ 포트 연결 성공: {SPIN_PORT}")

        # 2. ESC 암밍 (초기화)
        print("🔒 ESC 초기화 신호(R900) 전송 중... (5초 대기)")
        s_spin.write(b"R900\n")
        time.sleep(5)
        print("📢 ESC 준비 완료! (비프음 확인)")

        # 3. 소프트 스타트를 이용한 모터 가동
        print("\n🚀 [모터 가동 시작]")
        soft_start(s_spin, TARGET_PWM, step=2, delay=0.05)
        
        # 4. 일정 시간 회전 유지
        print("🔄 현재 속도로 5초간 회전을 유지합니다...")
        time.sleep(5)

        # 5. 모터 정지
        print("\n🛑 모터 정지 명령(R900) 전송")
        s_spin.write(b"R900\n")
        time.sleep(2) # 완전히 멈출 때까지 대기

        print("🎉 테스트가 성공적으로 종료되었습니다.")

    except serial.SerialException:
        print(f"❌ 포트 연결 실패! {SPIN_PORT}가 맞는지, 다른 프로그램이 사용 중이진 않은지 확인하세요.")
    except KeyboardInterrupt:
        print("\n🚨 사용자에 의한 강제 종료! 모터를 정지합니다.")
        if 's_spin' in locals() and s_spin.is_open:
            s_spin.write(b"R900\n")
    finally:
        if 's_spin' in locals() and s_spin.is_open:
            s_spin.write(b"R900\n") # 안전을 위해 한 번 더 900 전송
            s_spin.close()
            print("🔌 포트를 닫았습니다.")

if __name__ == "__main__":
    main()