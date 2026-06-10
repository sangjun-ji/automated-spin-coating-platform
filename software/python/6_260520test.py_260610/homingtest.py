import serial
import time

GRBL_PORT = 'COM3' 
BAUD_RATE = 115200
SEEK_SPEED = 800
PULL_OFF = 5 

# ==========================================
# 필수 함수 1: 알람 대기
# ==========================================
def wait_for_hardware_alarm(ser):
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'ALARM' in line or 'Reset to continue' in line:
            return True
        time.sleep(0.01)

# ==========================================
# 필수 함수 2: 이동 완료 대기 (철벽 방어 버전으로 수정됨 ⭐)
# ==========================================
def wait_until_idle(ser):
    time.sleep(0.5) # 기계가 명령을 받고 출발할 때까지 잠깐 기다려줌 (이게 없으면 출발도 전에 끝난 줄 암)
    ser.reset_input_buffer()
    while True:
        ser.write(b'?')
        status = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'Idle' in status:
            break
        time.sleep(0.05)

def run_hardware_safe_homing():
    try:
        s = serial.Serial(GRBL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        s.reset_input_buffer()
        
        print("--- [PASCAL] 하드웨어 브레이크($21=1) 기반 안전 호밍 ---")

        # 1. 하드 리밋 켜기
        s.write(b'\x18')
        time.sleep(1)
        s.write(b'$X\n')
        time.sleep(0.5)
        s.write(b'$21=1\n') 
        time.sleep(0.5)

        # 2. X축 탐색
        print("\n▶ [1/2] X축 센서 탐색 중... (- 방향)")
        s.write(f'G91 G1 X-2000 F{SEEK_SPEED}\n'.encode())
        wait_for_hardware_alarm(s)
        
        s.write(b'\x18')     
        time.sleep(1)
        s.write(b'$X\n')     
        time.sleep(0.5)
        s.write(b'$21=0\n')  
        time.sleep(0.5)
        
        print(f"  🏃 X축 {PULL_OFF}mm 후퇴...")
        s.write(f'G91 G1 X{PULL_OFF} F500\n'.encode())
        wait_until_idle(s)
        # ⭐ 여기서 X축 0점 선언을 뺐습니다! (이따 한꺼번에 함)

        # 3. Y축 탐색을 위해 방어막 다시 켜기
        s.write(b'$21=1\n') 
        time.sleep(0.5)

        # 4. Y축 탐색
        print("\n▶ [2/2] Y축 센서 탐색 중... (+ 방향)")
        s.write(f'G91 G1 Y2000 F{SEEK_SPEED}\n'.encode())
        wait_for_hardware_alarm(s)
        
        s.write(b'\x18')     
        time.sleep(1)
        s.write(b'$X\n')     
        time.sleep(0.5)
        s.write(b'$21=0\n')  
        time.sleep(0.5)
        
        print(f"  🏃 Y축 {PULL_OFF}mm 후퇴...")
        s.write(f'G91 G1 Y-{PULL_OFF} F500\n'.encode())
        wait_until_idle(s)

        # ==========================================
        # ⭐ 모든 리셋(\x18)이 끝났으므로, 이제 진짜 0점을 찍습니다!
        # ==========================================
        s.write(b'G92 X0 Y0\n')
        print("  ✅ [X축, Y축 0점 세팅 최종 완료!]")
        time.sleep(0.5)

        # ==========================================
        # 5. 최종 목적지 이동
        # ==========================================
        print("\n🎯 목적지로 이동 시작")
        s.write(b'G90\n') # 절대 좌표 모드

        print("  ▶ 1번 지점 (X200)")
        s.write(b'G1 X200 Y0 F2000\n')
        wait_until_idle(s)        
        s.write(b'G1 X0 Y0 F2000\n')
        wait_until_idle(s)
        s.write(b'G1 X200 Y0 F2000\n')
        wait_until_idle(s)
        s.write(b'G1 X0 Y0 F2000\n')
        wait_until_idle(s)
        s.write(b'G1 X0 Y-50 F2000\n')
        wait_until_idle(s)
        s.write(b'G1 X0 Y0 F2000\n')
        wait_until_idle(s)
        s.write(b'G1 X0 Y-50 F2000\n')
        wait_until_idle(s)
        s.write(b'G1 X0 Y0 F2000\n')
        wait_until_idle(s)
        
        print("\n🏁 모든 미션 완벽하게 종료!")

    except Exception as e:
        print(f"오류: {e}")
    finally:
        if 's' in locals() and s.is_open:
            s.write(b'$21=0\n') 
            s.close()

if __name__ == "__main__":
    run_hardware_safe_homing()