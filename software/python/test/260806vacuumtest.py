import time
import serial

# ==========================================
# 1. 포트 설정
# ==========================================
RELAY_PORT = 'COM20'   # 릴레이 보드 (진공 펌프)
BAUD_RELAY = 115200    

# ==========================================
# 2. 진공 펌프 전용 테스트 레시피
# ==========================================
PROCESS_RECIPE = [
    {"step": "진공 펌프 ON (5초 유지)", "pump": 1, "delay": 2.0},
    {"step": "진공 펌프 OFF (2초 대기)", "pump": 0, "delay": 2.0},
  
]

# ==========================================
# 3. 메인 실행 루프
# ==========================================
if __name__ == "__main__":
    print("=== 💨 VACUUM PUMP ONLY TEST SYSTEM ===")
    
    s_relay = None

    # 1. 진공 펌프 릴레이 연결
    try:
        s_relay = serial.Serial(RELAY_PORT, BAUD_RELAY, timeout=1)
        print("✅ 릴레이 보드 (COM20) 연결 성공!")
        print("⏳ 릴레이 보드 MCU 부팅 및 통신 안정화 대기 중 (3초)...")
        time.sleep(3.0)  # 핵심: 릴레이 보드가 DTR 리셋 후 부팅을 완료할 때까지 대기
    except Exception as e:
        print(f"❌ 릴레이 포트 연결 실패: {e}")
        exit(1)

    # 초기 가드 설정 (펌프 안전 OFF)
    if s_relay and s_relay.is_open:
        s_relay.write(b"P0\n")
        s_relay.flush()
        time.sleep(0.1)

    print("\n✨ 시스템 준비 완료! 진공 펌프 테스트를 시작합니다.\n")
    time.sleep(1)

    # 2. 레시피 실행
    try:
        for idx, task in enumerate(PROCESS_RECIPE):
            print(f"▶ [STEP {idx+1}] {task['step']}")

            # --- 진공 펌프 (Relay) 제어 ---
            if task.get('pump') is not None:
                pump_val = int(task['pump']) 
                if s_relay and s_relay.is_open:
                    cmd = f"P{pump_val}\n".encode('ascii')
                    s_relay.write(cmd)
                    s_relay.flush()  
                
                status_str = "ON (P1)" if pump_val == 1 else "OFF (P0)"
                print(f"   └ 💨 [진공 펌프] {status_str} 명령 전송 완료!")

            # --- Step Delay ---
            delay_time = task.get('delay', 0.1)
            if delay_time > 0:
                time.sleep(delay_time)

        print("\n🎉 모든 진공 펌프 테스트 공정이 성공적으로 완료되었습니다!")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단 (Ctrl+C)")
    finally:
        # 프로그램 종료/중단 시 안전 구동 (펌프 차단)
        if s_relay and s_relay.is_open:
            try:
                s_relay.write(b"P0\n")
                s_relay.flush()
                time.sleep(0.05)
                s_relay.close()
                print("💨 [진공 펌프] 안전 OFF 완료 및 시리얼 포트 닫기 성공")
            except Exception as e:
                print(f"⚠️ 릴레이 종료 중 에러: {e}")

        print("🔌 하드웨어 포트가 안전하게 종료되었습니다.")