import time
import serial

# ==========================================
# 1. 포트 및 통신 설정 (피펫만 사용)
# ==========================================
PIPETTE_PORT = 'COM13'   # 피펫 & 피펫 Z축 RS-485
BAUD_PIP     = 38400

P_ADDR   = 1    # 피펫 본체 (SADP20) 주소
P_Z_ADDR = 41   # 피펫 Z축 주소

# ==========================================
# 2. 피펫 RS-485 헬퍼 함수
# ==========================================
def send_pipette_oem(ser, addr, cmd_str):
    """피펫 OEM RS485 패킷 전송 및 응답 확인"""
    try:
        if ser and ser.is_open:
            ser.reset_input_buffer()

            header = 0xAA
            cmd_bytes = cmd_str.encode('ascii')
            frame = bytearray([header, addr, len(cmd_bytes)]) + cmd_bytes
            checksum = sum(frame) % 256
            frame.append(checksum)
            
            ser.write(frame)
            ser.flush()
            print(f"   └ 🧪 [피펫 RS485 (Addr:{addr})] 전송: {cmd_str}")
            
            time.sleep(0.2)
            
            if ser.in_waiting > 0:
                resp = ser.read_all()
                print(f"      └ 📩 [응답]: {resp.hex(' ')}")
            else:
                print(f"      ⚠️ [응답]: 수신 응답 없음")
                
            time.sleep(0.3)
    except Exception as e: 
        print(f"❌ 피펫 통신 에러: {e}")

# ==========================================
# 3. 피펫 테스트 전용 PROCESS_RECIPE
# ==========================================
PROCESS_RECIPE = [
    # 1. 초기화
{"step": "피펫 본체 초기화", "pip_z": None, "pipette": "It16000,100,2", "delay": 1.0},
    # 2. 팁 장착 및 해제 공정
{"step": "감지", "pip_z": None, "pipette":"Ld1,10000", "delay": 2.0},
{"step": "피펫 Z축 하강 (해제 위치)", "pip_z": "Zp100000,20000", "pipette": None, "delay": 2.0},
{"step": "피펫 Z축 하강 (해제 위치)", "pip_z": None, "pipette": "Ia15000,100,10", "delay": 2.0},
    
] 
    

# ==========================================
# 4. 메인 자동화 실행 루프 (피펫 전용)
# ==========================================
if __name__ == "__main__":
    print("=== 🧪 PIPETTE TEST RECIPE RUNNER ===")
    
    # 1. 피펫 시리얼 포트 연결
    try:
        s_pip = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        print("✅ 피펫 RS-485 시리얼 연결 완료!")
    except Exception as e:
        print(f"❌ 피펫 시리얼 포트 연결 실패: {e}")
        s_pip = None

    # 🛑 [수정 포인트 1] FMC4030, ODrive, 핫플레이트, 그리퍼 연결 및 초기화 로직 제거/스킵
    # - FMC4030 호밍 (home_fmc_axis) 스킵
    # - 핫플레이트 예열 (wait_for_hotplate) 스킵[cite: 6]
    # - 그리퍼 원점 탐색 스킵[cite: 6]
    # - ODrive 스핀코터 캘리브레이션 스킵[cite: 6]

    # 2. 피펫 Z축 상단 대피 (안전 동작)
    if s_pip and s_pip.is_open:
        print("\n🛡️ [안전] 피펫 Z축 최상단 안전 위치 대피...")
        send_pipette_oem(s_pip, P_Z_ADDR, "Zz50000")
        time.sleep(3.0)

    print("\n✨ 준비 완료! 피펫 테스트 레시피를 시작합니다.\n")
    time.sleep(1)

    # 3. 레시피 실행 루프
    try:
        for idx, task in enumerate(PROCESS_RECIPE):
            print(f"\n▶ [STEP {idx+1}] {task['step']}")

            # --- [수정 포인트 2] 피펫 제어 구문만 남기고 타 장비 제어 조건문 스킵 ---
            
            # 피펫 Z축 (Addr: 41)
            if task.get('pip_z') is not None and s_pip and s_pip.is_open:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])
                
            # 피펫 본체 SADP20 (Addr: 1)
            if task.get('pipette') is not None and s_pip and s_pip.is_open:
                send_pipette_oem(s_pip, P_ADDR, task['pipette'])

            # Step Delay
            delay_time = task.get('delay', 0.1)
            if delay_time > 0:
                time.sleep(delay_time)

        print("\n🎉 피펫 테스트 레시피 공정이 성공적으로 완료되었습니다!")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단 (Ctrl+C)")
    finally:
        if s_pip and s_pip.is_open:
            s_pip.close()
        print("🔌 피펫 포트 연결이 안전하게 종료되었습니다.")