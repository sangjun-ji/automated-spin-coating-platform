import time
import serial
import odrive
from odrive.utils import dump_errors

# ==========================================
# 0. 설정 파라미터
# ==========================================
TARGET_INIT_HOTPLATE_TEMP = 95.0
HOTPLATE_BOOST_OFFSET = 3.0

HOTPLATE_PORT = 'COM25'   # 핫플레이트 RS-485 포트
RELAY_PORT    = 'COM20'   # 릴레이 보드 (진공 펌프) 포트
BAUD_RELAY    = 115200    

# ODrive 상수 정의
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP = 2

odrv_start_pos = 0.0

# ==========================================
# 1. 핫플레이트 제어 헬퍼 함수
# ==========================================
def calculate_checksum(packet_str):
    """템코 T50 HSUM 8비트 체크섬 계산"""
    checksum_val = sum(ord(char) for char in packet_str)
    return f"{checksum_val & 0xFF:02X}"

def set_temperature(target_temp, address="01"):
    """핫플레이트 목표 온도(SV) 설정 패킷 생성"""
    temp_hex = f"{int(target_temp * 10):04X}"
    command_body = f"{address}DWR,01,0301,{temp_hex}"
    checksum = calculate_checksum(command_body)
    return f"\x02{command_body}{checksum}\r\n"

def get_current_temperature(ser, address="01"):
    """핫플레이트 현재 온도(PV) 읽기 함수"""
    try:
        command_body = f"{address}DRS,01,0001"
        checksum = calculate_checksum(command_body)
        packet = f"\x02{command_body}{checksum}\r\n"
        
        ser.reset_input_buffer()
        ser.write(packet.encode('ascii'))
        time.sleep(0.2)
        
        response = ser.read_all().decode('ascii', errors='ignore')
        
        if "DRS,OK" in response:
            clean_resp = response.strip('\x02\x03\r\n ')
            parts = clean_resp.split(',')
            
            if len(parts) >= 3 and parts[1] == "OK":
                raw_hex_temp = parts[2][:4]
                return int(raw_hex_temp, 16) / 10.0
                        
    except Exception as e:
        print(f"\n❌ [핫플레이트 통신 에러]: {e}")
        
    return None

def wait_for_hotplate(ser, target_temp, boost_offset=0.0, timeout=1800, check_interval=2.0):
    """급속 예열 적용 및 목표 온도 도달 대기 함수"""
    boost_temp = target_temp + boost_offset
    print(f"\n🚀 [핫플레이트] 예열 동작 개시!")
    print(f"   ├ 목표 온도: {target_temp:.1f}°C")
    
    set_packet = set_temperature(boost_temp)
    ser.write(set_packet.encode('ascii'))
    time.sleep(0.2)
    
    start_time = time.time()
    
    while True:
        curr_temp = get_current_temperature(ser)
        
        if curr_temp is not None:
            print(f"   ⏳ [예열 중...] 실제 온도: {curr_temp:.1f}°C / 목표: {target_temp:.1f}°C")
            
            if curr_temp >= target_temp:
                print(f"\n✅ [핫플레이트] 목표 온도 ({target_temp:.1f}°C) 도달 확인!")
                reset_packet = set_temperature(target_temp)
                ser.write(reset_packet.encode('ascii'))
                time.sleep(0.2)
                break
        else:
            print("   ⏳ [핫플레이트] 실제 온도 수신 대기 중...")

        if time.time() - start_time > timeout:
            raise TimeoutError("핫플레이트 예열 시간 초과")

        time.sleep(check_interval)

# ==========================================
# 2. 실시간 상태 출력 헬퍼 함수 (공정 경과 시간 표시)
# ==========================================
def monitor_spin_status(odrv, duration_sec, recipe_start_time, interval=0.2):
    """지정된 시간 동안 공정 경과 시간(초)과 실제 RPM을 터미널에 실시간 출력"""
    if odrv is None or duration_sec <= 0:
        return

    start_t = time.time()
    while (time.time() - start_t) < duration_sec:
        proc_time = time.time() - recipe_start_time
        try:
            curr_rpm = odrv.axis0.encoder.vel_estimate * 60.0
        except Exception:
            curr_rpm = 0.0

        print(f"\r   📊 [공정 시간: {proc_time:6.1f}초] 현재 스핀코터 속도: {curr_rpm:6.1f} RPM", end="", flush=True)
        time.sleep(interval)
    print()  # 줄바꿈

# ==========================================
# 3. ODrive 스핀코터 제어 함수 (연속 가속 지원)
# ==========================================
def control_spin_cooter(odrv, target_rpm, accel_time_sec=1.0, recipe_start_time=None):
    global odrv_start_pos

    if target_rpm > 0:
        target_rps = target_rpm / 60.0
        
        if accel_time_sec <= 0:
            accel_time_sec = 0.1

        # 💡 현재 속도와 목표 속도 차이(Delta RPS) 기반 가속도 계산
        try:
            current_rps = odrv.axis0.controller.input_vel
        except Exception:
            current_rps = 0.0

        delta_rps = abs(target_rps - current_rps)
        ramp_rate_rps = delta_rps / accel_time_sec if delta_rps > 0 else (target_rps / accel_time_sec)

        print(f"\n🌀 [스핀코터] {target_rpm} RPM 구동/변속 명령 (목표 가속 시간: {accel_time_sec}초)")
        print(f"   └ 📐 계산된 가속도(Ramp Rate): {ramp_rate_rps:.2f} RPS/s")
        
        odrv.axis0.encoder.config.use_index = False
        
        if not odrv.axis0.encoder.is_ready:
            print("   ❌ ODrive 엔코더 미준비 상태!")
            dump_errors(odrv)
            return False

        if current_rps == 0.0:
            odrv_start_pos = odrv.axis0.encoder.pos_estimate

        odrv.axis0.controller.config.vel_limit = 100.0
        odrv.axis0.controller.config.vel_limit_tolerance = 2.0
        odrv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        odrv.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        
        # 가속도 적용
        odrv.axis0.controller.config.vel_ramp_rate = ramp_rate_rps

        odrv.axis0.controller.config.vel_gain = 0.05
        odrv.axis0.controller.config.vel_integrator_gain = 0.2

        if odrv.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            odrv.axis0.controller.input_vel = 0.0
            odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.1)

        odrv.axis0.controller.input_vel = target_rps

    else:
        print(f"\n⏹️ [스핀코터] 정지 명령 수신 -> 감속 및 정밀 영점 안착 시작...")
        
        odrv.axis0.controller.input_vel = 0.0
        while abs(odrv.axis0.encoder.vel_estimate) > 0.5:
            proc_time = time.time() - recipe_start_time if recipe_start_time else 0.0
            curr_rpm = odrv.axis0.encoder.vel_estimate * 60.0
            print(f"\r   🛑 [공정 시간: {proc_time:6.1f}초] 감속 중... 현재 속도: {curr_rpm:6.1f} RPM", end="", flush=True)
            time.sleep(0.05)
        print()

        stage2_start = time.time()
        current_pos = odrv.axis0.encoder.pos_estimate
        relative_turns = current_pos - odrv_start_pos

        nearest_target_turns = round(relative_turns)
        target_pos = odrv_start_pos + nearest_target_turns

        odrv.axis0.controller.config.pos_gain = 30.0
        odrv.axis0.controller.config.vel_gain = 0.05
        odrv.axis0.controller.config.vel_integrator_gain = 0.2

        odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        odrv.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        odrv.axis0.controller.input_pos = target_pos

        while (time.time() - stage2_start) < 1.0:
            pos_err_deg = abs(odrv.axis0.encoder.pos_estimate - target_pos) * 360.0
            cur_vel = abs(odrv.axis0.encoder.vel_estimate)
            if pos_err_deg <= 0.5 and cur_vel < 0.05:
                break
            time.sleep(0.01)

        final_deg = ((odrv.axis0.encoder.pos_estimate - odrv_start_pos) % 1.0) * 360.0
        if final_deg > 180.0: final_deg -= 360.0
        
        print(f"   ✅ [스핀코터] 원점 복귀 완료! (최종 오차: {final_deg:.2f}°)")
        odrv.axis0.requested_state = AXIS_STATE_IDLE

    return True

# ==========================================
# 4. 레시피 설정 (2단계 페로브스카이트 코팅)
# ==========================================
PROCESS_RECIPE = [
    {"step": "핫플레이트 온돌 설정 (30°C)", "hotplate":105.0, "pump": None, "spin": None, "delay": 1.0},
    {"step": "진공 펌프 ON", "hotplate": None, "pump": 1, "spin": None, "delay": 2.0},
    
    # 🧪 1차 용액 코팅: 0 -> 1000 RPM (2초 가속 후 8초 유지)
    {"step": "스핀코터 1단계 (00 RPM, 2초 가속, 8초 유지)", "hotplate": None, "pump": None, "spin": 5000, "accel_time": 2.0, "spin_time": 30.0, "delay": 0.0},
    
\
    {"step": "스핀코터 정지", "hotplate": None, "pump": None, "spin": 0, "delay": 1.0},
    {"step": "진공 펌프 OFF", "hotplate": None, "pump": 0, "spin": None, "delay": 1.0},
]

# ==========================================
# 5. 메인 실행 루프
# ==========================================
if __name__ == "__main__":
    print("=== 🧪 스핀코터 & 핫플레이트 & 진공 펌프 통합 제어 시스템 가동 ===")
    
    s_hotplate = None
    s_relay = None
    odrv0 = None

    # 1. 시리얼 연결
    try:
        s_hotplate = serial.Serial(
            HOTPLATE_PORT, 
            baudrate=9600, 
            bytesize=serial.SEVENBITS, 
            parity=serial.PARITY_EVEN, 
            stopbits=serial.STOPBITS_ONE, 
            timeout=2
        )
        s_relay = serial.Serial(RELAY_PORT, BAUD_RELAY, timeout=1)
        print("✅ 핫플레이트 및 릴레이 시리얼 연결 완료!")
    except Exception as e:
        print(f"⚠️ 시리얼 포트 연결 실패: {e}")

    # 2. ODrive 연결
    try:
        print("🔍 ODrive 스핀코터 연결 중...")
        odrv0 = odrive.find_any()
        if hasattr(odrv0, 'clear_errors'): 
            odrv0.clear_errors()
            
        if not odrv0.axis0.encoder.is_ready:
            print("⚙️ [ODrive] 엔코더 영점 미설치 감지 -> 자동 캘리브레이션 진행 중...")
            odrv0.axis0.encoder.config.calib_range = 0.5
            odrv0.axis0.requested_state = 7
            
            while odrv0.axis0.current_state != 1: 
                time.sleep(0.1)
            time.sleep(0.5)

        if odrv0.axis0.encoder.is_ready:
            print("✅ ODrive 스핀코터 자동 준비 완료!")
        else:
            print("❌ ODrive 엔코더 정렬 실패!")

    except Exception as e:
        print(f"⚠️ ODrive 연결 실패: {e}")

    # 초기 가드 설정
    if s_relay and s_relay.is_open:
        s_relay.write(b"P0\n")
        s_relay.flush()
        time.sleep(0.05)

    # 3. 핫플레이트 예열 단계 실행
    if s_hotplate and s_hotplate.is_open:
        wait_for_hotplate(
            ser=s_hotplate, 
            target_temp=TARGET_INIT_HOTPLATE_TEMP, 
            boost_offset=HOTPLATE_BOOST_OFFSET
        )
    else:
        print("\n⚠️ 핫플레이트 시리얼 미연결로 예열 로직을 생략합니다.")

    print("\n✨ 모든 설정 완료! 통합 제어 레시피를 시작합니다.\n")
    time.sleep(1)

    # 4. 레시피 실행 루프 (전체 공정 시간 측정 시작)
    try:
        recipe_start_time = time.time()

        for idx, task in enumerate(PROCESS_RECIPE):
            proc_time_now = time.time() - recipe_start_time
            print(f"\n▶ [STEP {idx+1}] {task['step']} (공정 시간: {proc_time_now:.1f}s)")

            # 핫플레이트 제어
            if task.get('hotplate') is not None and s_hotplate and s_hotplate.is_open:
                packet = set_temperature(task['hotplate'])
                s_hotplate.write(packet.encode('ascii'))
                print(f"   └ ♨️ [핫플레이트] {task['hotplate']}°C 설정 전송")

            # 진공 펌프 (릴레이) 제어
            if task.get('pump') is not None:
                pump_val = int(task['pump']) 
                if s_relay and s_relay.is_open:
                    cmd = f"P{pump_val}\n".encode('ascii')
                    s_relay.write(cmd)
                    s_relay.flush()  
                time.sleep(0.1)
                print(f"   └ 💨 [진공 펌프] {'ON (P1)' if pump_val == 1 else 'OFF (P0)'} 전송 완료!")

            # ODrive 스핀코터 ON/OFF 제어 및 실시간 모니터링
            if task.get('spin') is not None and odrv0 is not None:
                target_rpm = task['spin']
                accel_time_val = task.get('accel_time', 1.0)
                spin_duration = task.get('spin_time', 0.0)

                control_spin_cooter(odrv0, target_rpm, accel_time_sec=accel_time_val, recipe_start_time=recipe_start_time)
                
                if target_rpm > 0 and spin_duration > 0:
                    print(f"   ⏳ [스핀 유지시간] {spin_duration:.1f}초 동안 속도 실시간 측정 중...")
                    monitor_spin_status(odrv0, spin_duration, recipe_start_time)

            delay_time = task.get('delay', 0.1)
            if delay_time > 0:
                time.sleep(delay_time)

        print(f"\n🎉 레시피 공정이 완료되었습니다! (총 공정 소요 시간: {time.time() - recipe_start_time:.1f}초)")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단 (Ctrl+C)")
    finally:
        if odrv0 is not None:
            try:
                odrv0.axis0.requested_state = AXIS_STATE_IDLE
            except: pass

        if s_relay and s_relay.is_open:
            try:
                s_relay.write(b"P0\n")
                s_relay.flush()
                time.sleep(0.05)
                s_relay.close()
                print("💨 [진공 펌프] 안전 OFF 완료")
            except: pass

        if s_hotplate and s_hotplate.is_open:
            try:
                s_hotplate.close()
                print("♨️ [핫플레이트] 시리얼 포트 닫기 완료 (설정 온도 유지됨)")
            except: pass