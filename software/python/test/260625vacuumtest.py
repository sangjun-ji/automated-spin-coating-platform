import serial
import time

# --- 설정 ---
SAFETY_PORT = 'COM18'  # 수정하신 포트 번호
BAUD_SAFE = 115200

def main():
    print("--- 🗜️ NO방식 진공 펌프 & 밸브 제어 테스트 ---")
    try:
        # 1. 시리얼 연결
        s_safe = serial.Serial(SAFETY_PORT, BAUD_SAFE, timeout=1)
        time.sleep(2) # 보드 연결 안정화 대기
        print(f"✅ 포트 연결 성공: {SAFETY_PORT}")

        # 2. 진공 시스템 시작 (펌프 ON -> 밸브 ON)
        print("\n💨 1. 펌프 가동 (P1)")
        s_safe.write(b"P1\n") 
        time.sleep(0.5) # 펌프가 돌기 시작할 시간 확보
        
        print("💨 2. 밸브 가동 (V1) - 진공 흡착 시작!")
        s_safe.write(b"V1\n")
        
        print("⏳ 5초간 진공 상태를 유지합니다. 기판이 잘 붙어 있는지 확인하세요.")
        time.sleep(5) 

        # 3. 진공 해제 (밸브 OFF -> 펌프 OFF)
        print("\n🛑 3. 밸브 해제 (V0)")
        s_safe.write(b"V0\n") 
        time.sleep(0.5) # 밸브가 먼저 닫혀서 대기압 차단
        
        print("🛑 4. 펌프 정지 (P0)")
        s_safe.write(b"P0\n")
        
        print("\n🎉 테스트가 성공적으로 종료되었습니다.")

    except serial.SerialException:
        print(f"❌ 포트 연결 실패! {SAFETY_PORT}가 맞는지 확인하세요.")
    except KeyboardInterrupt:
        print("\n🚨 강제 종료! 안전을 위해 시스템을 초기화합니다.")
        if 's_safe' in locals() and s_safe.is_open:
            s_safe.write(b"V0\n") # 밸브 먼저 닫고
            s_safe.write(b"P0\n") # 펌프 정지
    finally:
        if 's_safe' in locals() and s_safe.is_open:
            s_safe.write(b"V0\n")
            s_safe.write(b"P0\n")
            s_safe.close()
            print("🔌 포트를 닫았습니다.")

if __name__ == "__main__":
    main()