import serial
import time
import sys

class GantrySequenceTest:
    def __init__(self, port, baudrate=115200):
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2) 
            self.ser.flushInput()
            print(f"✅ 연결 성공: {port}")
        except Exception as e:
            print(f"❌ 연결 실패: {e}")
            sys.exit()

    def send(self, cmd):
        if self.ser is None: return False
        self.ser.write(f"{cmd}\n".encode())
        print(f">> 전송: {cmd}")
        while True:
            raw_data = self.ser.readline()
            if not raw_data: continue
            line = raw_data.decode().strip()
            if line == 'ok': return True
            if 'error' in line.lower() or 'alarm' in line.lower():
                print(f"❗ 응답 에러: {line}")
                return False

    def setup(self):
        print("\n--- [3축 초기 설정 적용] ---")
        self.send("$X") # 알람 해제

        # 스텝/속도/가속도 설정 (Z축: 리드 2mm, 8분주 기준)
        self.send("$100=16.842")  # X 스텝
        self.send("$101=16.842")  # Y 스텝
        self.send("$102=800.000") # Z 스텝

        self.send("$110=15000")   # X 최대로직속도
        self.send("$111=15000")   # Y 최대로직속도
        self.send("$112=5000")    # Z 최대로직속도

        self.send("$120=500")     # X 가속도
        self.send("$121=200")     # Y 가속도
        self.send("$122=200")     # Z 가속도

        self.send("G92 X0 Y0 Z0") # 현재 위치 원점 설정
        self.send("G21")          # mm 단위
        self.send("G90")          # 절대 좌표
        print("✅ 설정 완료! X -> Y -> Z 순차 이동 준비.\n")

    def run_test(self):
        # 속도 정의 (mm/min)
        x_speed = 12000  # 200mm/s
        y_speed = 4800   # 80mm/s
        z_speed = 4800   # 80mm/s

        print(f"🚀 테스트 시작 (순서: X -> 1초 휴식 -> Y -> 1초 휴식 -> Z)")

        # 1. X축 400mm 이동
        print(f"1. X축 이동 중 (400mm @ {x_speed}mm/min)...")
        self.send(f"G1 X400 F{x_speed}") 
        time.sleep(3) # 이동(2초) + 대기(1초)

        print("⏸️ 1초간 정지...")
        
        # 2. Y축 80mm 이동
        print(f"2. Y축 이동 중 (80mm @ {y_speed}mm/min)...")
        self.send(f"G1 Y80 F{y_speed}") 
        time.sleep(2) # 이동(1초) + 대기(1초)

        print("⏸️ 1초간 정지...")

        # 3. Z축 50mm 이동
        print(f"3. Z축 이동 중 (50mm @ {z_speed}mm/min)...")
        self.send(f"G1 Z50 F{z_speed}") 
        time.sleep(1) # 이동(~0.6초) + 여유

        print(f"\n📍 최종 위치 도달: X=400, Y=80, Z=50")

if __name__ == "__main__":
    robot = GantrySequenceTest(port='COM3') 
    robot.setup()
    
    input("⚠️ 엔터를 누르면 [X -> Y -> Z] 순차 테스트를 시작합니다...")
    
    try:
        robot.run_test()
    except KeyboardInterrupt:
        print("\n🛑 중단됨")
    finally:
        if robot.ser:
            robot.ser.close()
            print("🔌 시리얼 연결 종료")