import serial
import time

PORT = 'COM11'
BAUD_RATE = 115200

# 사용자가 제공한 데이터 기반 (PWM, RPM)
known_pwm = [0, 20, 30, 50, 80, 100, 150, 200, 250]
known_rpm = [1500, 2280, 2700, 3420, 4500, 5250, 7150, 9900, 9900]

def get_initial_pwm_simple(target_rpm):
    """라이브러리 없이 선형 보간 수행 (데이터 사이의 값을 추정)"""
    # 범위를 벗어나는 경우 처리
    if target_rpm <= known_rpm[0]: return known_pwm[0]
    if target_rpm >= known_rpm[-1]: return known_pwm[-1]
    
    # 데이터 사이의 값을 선형적으로 계산
    for i in range(len(known_rpm) - 1):
        if known_rpm[i] <= target_rpm <= known_rpm[i+1]:
            # 선형 보간 공식: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            x1, x2 = known_rpm[i], known_rpm[i+1]
            y1, y2 = known_pwm[i], known_pwm[i+1]
            pwm = y1 + (target_rpm - x1) * (y2 - y1) / (x2 - x1)
            return int(pwm)
    return 50 # 기본값

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"시스템 연결 성공: {PORT}")
    time.sleep(2)
except Exception as e:
    print(f"연결 실패 (포트 번호나 연결 상태 확인): {e}")
    exit()

def run_optimized_control(target_rpm_list):
    current_pwm = 50
    
    for target_rpm in target_rpm_list:
        # 데이터 기반으로 최적의 시작 PWM 계산
        current_pwm = get_initial_pwm_simple(target_rpm)
        print(f"\n[목표: {target_rpm} RPM] 시작 PWM: {current_pwm}")
        
        ser.write(f"{current_pwm}\n".encode())
        start_time = time.time()
        
        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        detected_rpm = float(line)
                        error = target_rpm - detected_rpm
                        
                        # 제어 로직: 오차에 따라 PWM 가감
                        if abs(error) > 200:
                            adj = 3
                        elif abs(error) > 50:
                            adj = 1
                        else:
                            adj = 0
                        
                        if error > 0: current_pwm = min(current_pwm + adj, 200)
                        else: current_pwm = max(current_pwm - adj, 0)

                        ser.write(f"{int(current_pwm)}\n".encode())
                        print(f"현재: {detected_rpm:6.0f} | 목표: {target_rpm} | PWM: {int(current_pwm)}")
                        
                    except ValueError: continue
            time.sleep(0.1)

try:
    run_optimized_control([2000, 3000, 5000])
finally:
    ser.write(b"0\n")
    ser.close()
    print("\n테스트 종료")