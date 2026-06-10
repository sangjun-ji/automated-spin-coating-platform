import serial
import time
import sys

GRBL_PORT = 'COM3'
BAUD_GRBL = 115200

def wait_for_x_alarm(ser):
    timeout_start = time.time()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"   [GRBL 응답]: {line}")
            if any(k in line for k in ['ALARM', 'Alarm', 'Reset', 'error:']):
                return True
        if time.time() - timeout_start > 15.0: 
            print("\n🚨 [위험] 15초 초과! 강제 종료!")
            sys.exit(1)
        time.sleep(0.005)

def main():
    try:
        print("--- [PASCAL] X축 원점 센서 단독 호밍 테스트 ---")
        s = serial.Serial(GRBL_PORT, BAUD_GRBL, timeout=1)
        time.sleep(2) 

        print("\n▶ [1] GRBL 보드 초기화 및 락 해제 ($X)")
        s.write(b'\x18\n'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.5)
        
        print("▶ [2] 하드웨어 리미트 센서 활성화 ($21=1)")
        s.write(b'$21=1\n'); time.sleep(0.5); s.reset_input_buffer()

        print("\n▶ [3] X축 센서를 향해 우측(또는 좌측)으로 이동 시작...")
        s.write(b'G91 G1 X2000 F1000\n') # X축 이동
        
        if wait_for_x_alarm(s):
            print("\n💥 X축 센서 충돌 감지 성공!")
            
            print("▶ [4] 충돌 후 안전 거리 하강 및 0점 세팅")
            time.sleep(0.1)
            s.write(b'\x18\n'); time.sleep(1); s.write(b'$X\n'); time.sleep(0.5); s.write(b'$21=0\n'); time.sleep(0.5)
            s.write(b'G91 G1 X-5 F500\n') 
            time.sleep(2)
            s.write(b'G92 X0\n')
            print("✅ X축 0점 세팅 완료!")

    except Exception as e:
        print(f"\n❌ 에러 발생: {e}")
    finally:
        if 's' in locals() and s.is_open:
            s.write(b'$21=0\n') 
            s.close()
        print("🔌 포트 닫힘.")

if __name__ == '__main__':
    main()