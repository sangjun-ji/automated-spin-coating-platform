import socket
import json
import time
import threading
import serial
import odrive
from odrive.utils import dump_errors

# ==========================================
# 0. 공정 초기화 설정 파라미터
# ==========================================
TARGET_INIT_HOTPLATE_TEMP = 29.0
HOTPLATE_BOOST_OFFSET = 0.0     

# ==========================================
# 1. 포트 및 통신 설정
# ==========================================
FMC_HOST = '127.0.0.1'  # FMC4030 IP 주소[cite: 8]
FMC_PORT = 50000        # FMC4030 포트[cite: 8]

GRIPPER_PORT  = 'COM12'   # 그리퍼 RS-485[cite: 8]
PIPETTE_PORT  = 'COM13'   # 피펫 & 피펫 Z축 RS-485[cite: 8]
HOTPLATE_PORT = 'COM25'   # 핫플레이트 RS-485[cite: 8]
RELAY_PORT    = 'COM20'   # 릴레이 보드 (진공 펌프)[cite: 8]

BAUD_GRIP  = 115200      # 그리퍼 보레이트[cite: 8]
BAUD_PIP   = 38400       # 피펫 보레이트[cite: 8]
BAUD_RELAY = 115200      # 릴레이 보레이트[cite: 8]

P_ADDR   = 1    # 피펫 본체 (SADP20) 주소[cite: 8]
P_Z_ADDR = 41   # 피펫 Z축 주소[cite: 8]

# ODrive 상수 정의[cite: 8]
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP = 2

odrv_start_pos = 0.0

# ==========================================
# 1-0. ODrive USB 링크 안전 설정
# ==========================================
# [배경]
# ODrive v3.x의 네이티브 USB는 BLDC 스위칭 노이즈에 취약하고, 긴 유휴 뒤
# 재개될 때 링크가 죽는 경우가 있다(USB 아이솔레이터는 서스펜드/리줌 신호를
# 제대로 통과시키지 못하는 제품이 많다). 공정 중간에 링크가 끊기면
# 스핀 정지 명령을 보낼 수 없어 모터가 계속 회전하는 위험이 있다.
#
# [대응 3단계]
#  1) 백그라운드 keepalive 스레드가 계속 링크를 두드려 유휴 상태를 만들지 않는다.
#  2) 스핀 중에는 ODrive 하드웨어 워치독을 켜서, 통신이 끊기면 모터가
#     스스로 IDLE로 떨어지게 한다(폭주 방지 - 가장 중요한 안전장치).
#  3) 공정 시작 전에 링크 안정성을 미리 시험해서, 불안하면 시료를 쓰기 전에
#     중단한다("중간에 실패"보다 "시작 전에 실패"가 훨씬 낫다).

ODRV = None                        # 현재 유효한 ODrive 핸들 (재연결 시 갱신)
ODRV_LOCK = threading.RLock()      # 메인 스레드 <-> keepalive 스레드 동시접근 보호

ODRIVE_MAX_RETRY = 3               # 통신 실패 시 재연결 후 재시도 횟수
ODRIVE_FIND_TIMEOUT = 15.0         # 최초 연결 탐색 타임아웃(초)
ODRIVE_RECONNECT_TIMEOUT = 5.0     # 재연결 탐색 타임아웃(초, 짧게)
# 1-0 설정 블록에 추가
ODRIVE_CALIB_TIMEOUT = 180.0   # 엔코더 캘리브레이션 대기(초)

# --- keepalive 스레드 ---
ODRIVE_KEEPALIVE_INTERVAL = 0.5    # 링크 유지 핑 주기(초)

# --- 하드웨어 워치독 (스핀 중에만 활성화) ---
# 통신이 이 시간 동안 끊기면 ODrive가 스스로 축을 IDLE로 내려 모터를 정지시킨다.
# keepalive 주기(0.5초)의 6배 여유 -> 일시적 지연으로는 오동작하지 않는다.
ODRIVE_USE_WATCHDOG = True
ODRIVE_WATCHDOG_TIMEOUT = 3.0

# --- 공정 시작 전 링크 안정성 시험 ---
ODRIVE_PREFLIGHT_ENABLE = True
ODRIVE_PREFLIGHT_DURATION = 5.0    # 시험 시간(초)
ODRIVE_PREFLIGHT_MAX_FAIL = 0      # 허용 실패 횟수 (0 = 단 한 번도 실패하면 안 됨)
ODRIVE_PREFLIGHT_ABORT = True      # 시험 실패 시 공정을 아예 시작하지 않음

# --- 스핀 속도 검증 ---
ODRIVE_VERIFY_SPIN = True
ODRIVE_SPIN_VERIFY_TOLERANCE = 0.15   # 목표 대비 허용 오차 비율 (15%)

# 내부 상태 (직접 수정하지 말 것)
_odrv_keepalive_thread = None
_odrv_keepalive_stop = threading.Event()
_odrv_watchdog_active = False
_odrv_link_lost_count = 0

# ==========================================
# 1-1. 핸드셰이크(정지/완료 확인) 사용 설정
# ==========================================
# 장비별로 실제 "정지/완료 확인" 통신 테스트가 끝나면 True로 바꿔서 사용.
# False인 장비는 기존 방식(명령 전송 후 task['delay']만 대기)으로 그대로 동작함.
HANDSHAKE_OK = {
    "gripper": True,   # 그리퍼 (Modbus RTU, Holding Register 10 Status)
    "pip_z":   True,   # 피펫 Z축 (OEM 프로토콜, Addr 41)
    "pipette": True,   # 피펫 본체 SADP20 (OEM 프로토콜, Addr 1)
}

# 핸드셰이크 대기 타임아웃(초) - 장비별로 필요시 조정
GRIPPER_HANDSHAKE_TIMEOUT = 10.0
PIPZ_HANDSHAKE_TIMEOUT    = 15.0
PIPETTE_HANDSHAKE_TIMEOUT = 15.0

# --- 피펫 비동기(arm) 명령 --------------------------------------------------
# Ld(액체 감지)는 "감지 기능을 켜두는" 명령이다. 매뉴얼 권장 절차:
#   시약 표면 5mm 위까지 하강 -> Ld 전송 -> 계속 하강 -> 감지되면 Z축 정지
# 즉 Ld를 켠 뒤 Z축이 내려가야 압력 변화가 생겨 감지된다.
# 여기서 완료를 기다리면 Z축이 멈춰 있는 채로 타임아웃(status 22)만 소진되고
# 감지가 꺼진 뒤에 하강이 시작되어 액체를 그냥 지나친다.
# 따라서 이런 명령은 "접수 확인(ack)"만 하고 즉시 다음 스텝으로 넘어간다.
PIPETTE_ASYNC_PREFIXES = ("Ld",)

# 피펫 명령 전송 후 응답 대기 방식.
# 예전에는 무조건 0.2초를 쉬었지만, 실제 응답은 38400bps에서 보통 10~30ms면 온다.
# 아래 타임아웃까지 "응답이 올 때까지만" 기다리므로 스텝당 ~0.18초를 절약한다.
PIPETTE_ACK_TIMEOUT = 0.3     # 명령 접수 응답 최대 대기(초)

# 상태 조회('?') 응답 최대 대기(초).
# ※ ser.read(N)은 N바이트가 다 찰 때까지 시리얼 timeout(1초)만큼 블로킹된다.
#    응답은 5바이트뿐이므로 반드시 in_waiting 기반 논블로킹 수신을 써야 한다.
PIPETTE_STATUS_TIMEOUT = 0.25
# 상태 폴링 주기(초)
PIPETTE_POLL_INTERVAL  = 0.03
PIPETTE_ACK_DELAY  = 0.2      # (호환용, 현재는 미사용)
# 피펫 명령 전송 후 꼬리 대기(초). 핸드셰이크가 실제 완료를 확인하므로 짧게 유지.
# 통신이 불안정하면 0.3 정도로 올릴 것.
PIPETTE_CMD_TAIL_DELAY = 0.02

# 액체 감지 상태 코드
PIP_STATUS_IDLE        = 0
PIP_STATUS_BUSY        = 1
PIP_STATUS_SUCCESS     = 2
PIP_STATUS_LIQUID_FOUND = 4    # 액체 감지됨
PIP_STATUS_TIMEOUT     = 22    # 타임아웃 (액체 미감지)

# 그리퍼 Modbus 원시 송수신 hex를 출력할지 여부 (진단용)
# 통신이 정상 확인되면 False로 되돌려서 로그를 깔끔하게 유지
GRIPPER_DEBUG = False

# 시작 시 그리퍼 상태 레지스터 원시 응답을 몇 번 찍어볼지 (0이면 생략)
GRIPPER_DEBUG_PROBE_COUNT = 3

# ==========================================
# 1-2. 공정 속도 튜닝 파라미터
# ==========================================
# 핸드셰이크가 실제 완료를 확인하므로, 예전의 "무조건 기다리기" 시간은 줄일 수 있다.
# 문제가 생기면 각 값을 다시 올리면 된다.

# --- FMC4030 축 이동 ---
# 예전: 이동 명령 후 무조건 0.3초 대기 -> 0.2초 간격 폴링 -> 끝나고 또 0.2초 대기
#       축 이동 1회당 0.5초 + 폴링 오차. 68회면 40초가 그냥 날아간다.
# 지금: 빠르게 폴링하되, 축이 실제로 움직이기 시작한 것을 확인한 뒤
#       정지 신호를 연속 2회 받아야 완료로 인정한다(조기 완료 방지).
FMC_POLL_INTERVAL   = 0.03    # 정지 확인 폴링 주기(초)
FMC_MOTION_GRACE    = 0.30    # 축이 아직 안 움직였을 때 출발을 기다리는 최대 시간(초)
FMC_STOP_CONFIRM    = 2       # 정지 신호를 연속 N회 받아야 완료
FMC_TAIL_DELAY      = 0.0     # 이동 완료 후 추가 대기(초)

# --- 그리퍼 ---
GRIPPER_CMD_DELAY   = 0.02    # 명령 write 후 대기(예전 0.1초)

# --- 레시피 delay 일괄 조정 ---
# 각 스텝의 "delay" 값에 이 배율을 곱한다.
#   1.0 = 레시피에 적힌 그대로
#   0.5 = 절반
#   0.0 = 전부 생략 (핸드셰이크만 믿고 진행)
STEP_DELAY_SCALE    = 1

# --- 피펫 Z축 위치 로깅 ---
# Zg(팁 장착) 후 실제 스톨 지점을 알아야 Zp 접근 높이를 튜닝할 수 있다.
# True로 두면 pip_z 동작이 끝날 때마다 현재 Z 위치(um)를 출력한다.
PIPZ_LOG_POSITION   = True

# --- 그리퍼 정지 판정 파라미터 -------------------------------------------
# ※ 실측 결과 이 펌웨어의 Status bit2(Position Control)는 "이동 중"이 아니라
#    "위치제어 모드 유지 중"을 의미해서 정지해도 0으로 내려가지 않는다.
#    따라서 정지 판정은 reg13(Motor Velocity) + reg14(Finger Position)으로 한다.
GRIPPER_SETTLE_COUNT   = 3     # 속도0 & 위치불변이 연속 N회면 정지로 확정
GRIPPER_POS_EPSILON    = 2     # 위치 불변으로 볼 허용 변동폭 (0~1000 스케일)
GRIPPER_POS_TOLERANCE  = 15    # 목표 도달로 볼 허용 오차 (0~1000 스케일)
GRIPPER_MIN_WAIT       = 0.10  # 모터가 출발할 시간을 주는 최소 대기(초)
GRIPPER_MOTION_GRACE   = 1.0   # 아직 목표에 도달하지 않았는데 모터가 안 움직일 때,
                               # 출발을 기다려주는 최대 시간(초). 이 시간이 지나면
                               # "움직이지 않고 멈춰 있음"으로 확정한다.

# ==========================================
# 2. RS-485 / 핫플레이트 / 릴레이 헬퍼 함수
# ==========================================
def crc16(data):
    """그리퍼 CRC16 계산 함수"""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc.to_bytes(2, 'little')

def send_gripper_cmd(ser, command, value=0):
    """그리퍼 제어 명령 전송 함수"""
    try:
        SLAVE_ID = 1
        frame = bytearray([SLAVE_ID, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04])
        frame += bytearray([(command >> 8) & 0xFF, command & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
        frame += crc16(frame)
        ser.write(frame)
        print(f"   └ 🤏 [그리퍼] 명령: {command}, Value: {value}")
        time.sleep(GRIPPER_CMD_DELAY)
    except Exception as e:
        print(f"❌ 그리퍼 에러: {e}")

def _to_signed16(v):
    """16비트 무부호 값을 부호 있는 값으로 변환 (속도/각도는 음수 가능)"""
    return v - 0x10000 if v & 0x8000 else v


def query_gripper_status(ser, debug=False):
    """
    그리퍼 Modbus RTU 0x03(레지스터 읽기)으로 Holding Register 10~14 조회.

    반환: dict. 응답이 없거나 파싱 실패 시 None.
      - status     : Address 10 (비트별 동작 상태)
      - motor_pos  : Address 11 (모터 절대 각도)
      - current    : Address 12 (모터 전류, mA)
      - velocity   : Address 13 (모터 속도, rpm)  ★ 정지 판정의 핵심
      - finger_pos : Address 14 (0=완전히 닫힘 ~ 1000=완전히 열림)

    ※ ser.read(N)은 N바이트가 다 찰 때까지 timeout(1초)만큼 블로킹되므로,
      in_waiting 기반 논블로킹 수신으로 변경해 폴링 주기를 확보함.
    """
    try:
        if not (ser and ser.is_open):
            return None
        ser.reset_input_buffer()
        SLAVE_ID = 1
        # 함수코드 0x03, 시작주소 10(0x000A), 레지스터 개수 5 (매뉴얼 C 예제와 동일)
        frame = bytearray([SLAVE_ID, 0x03, 0x00, 0x0A, 0x00, 0x05])
        frame += crc16(frame)
        ser.write(frame)
        ser.flush()

        # --- 논블로킹 수신 (최대 0.3초) ---
        resp = b''
        t0 = time.time()
        while time.time() - t0 < 0.3:
            if ser.in_waiting > 0:
                time.sleep(0.03)               # 프레임 전체가 도착할 시간을 줌
                resp = ser.read(ser.in_waiting)
                break
            time.sleep(0.005)

        if debug:
            print(f"\n      [그리퍼 RAW] 송신: {frame.hex(' ')}")
            print(f"      [그리퍼 RAW] 수신: {resp.hex(' ') if resp else '(무응답)'}")

        if not resp:
            return None

        # --- RS-485 어댑터 에코 제거 (송신 프레임이 그대로 되돌아오는 경우) ---
        if resp.startswith(bytes(frame)):
            resp = resp[len(frame):]
            if debug:
                print(f"      [그리퍼 RAW] 에코 제거 후: {resp.hex(' ') if resp else '(없음)'}")
            if not resp:
                return None

        # --- Modbus 예외 응답 (0x83) ---
        if len(resp) >= 3 and resp[0] == SLAVE_ID and resp[1] == 0x83:
            if debug:
                print(f"      [그리퍼 RAW] ⚠️ Modbus 예외 응답! 예외코드 = {resp[2]}")
            return None

        # --- 정상 응답: [id][0x03][byte_count][data...][crc_lo][crc_hi] ---
        if len(resp) >= 5 and resp[0] == SLAVE_ID and resp[1] == 0x03:
            byte_count = resp[2]
            if byte_count >= 2 and len(resp) >= 3 + byte_count:
                info = {
                    "status":     (resp[3] << 8) | resp[4],   # reg 10
                    "motor_pos":  None,
                    "current":    None,
                    "velocity":   None,
                    "finger_pos": None,
                }
                if byte_count >= 10:   # reg 10~14 전부 수신된 경우
                    info["motor_pos"]  = _to_signed16((resp[5]  << 8) | resp[6])   # reg 11
                    info["current"]    = (resp[7]  << 8) | resp[8]                 # reg 12
                    info["velocity"]   = _to_signed16((resp[9]  << 8) | resp[10])  # reg 13
                    info["finger_pos"] = (resp[11] << 8) | resp[12]                # reg 14

                if debug:
                    print(f"      [그리퍼 PARSE] status=0b{info['status']:016b} "
                          f"vel={info['velocity']} finger={info['finger_pos']} "
                          f"cur={info['current']}mA")
                return info

        return None
    except Exception as e:
        print(f"❌ 그리퍼 상태 조회 에러: {e}")
        return None


def wait_for_gripper_stop(ser, target_pos=None,
                          timeout=GRIPPER_HANDSHAKE_TIMEOUT, poll_interval=0.05,
                          settle_count=GRIPPER_SETTLE_COUNT, debug=False):
    """
    그리퍼가 완전히 정지할 때까지 대기.

    [판정 기준]
      Status bit2는 이 펌웨어에서 "위치제어 모드 유지 중"을 의미해 정지 후에도
      1로 남아 있으므로 사용하지 않는다. 대신
        - reg13 (Motor Velocity) == 0
        - reg14 (Finger Position)이 GRIPPER_POS_EPSILON 이내로 불변
      이 두 조건이 연속 settle_count회 만족되면 정지로 확정한다.

      단, 명령 직후 모터가 아직 출발하지 않아 속도가 0인 순간을 "정지"로 오인하면
      그리퍼가 움직이는 중에 갠트리가 먼저 출발하는 위험이 있으므로,
        - 최소 GRIPPER_MIN_WAIT초는 무조건 관찰하고
        - 한 번도 움직임이 없었다면, 이미 목표 위치에 있는 경우가 아닌 한
          GRIPPER_MOTION_GRACE초까지 출발을 기다린다.

    [반환]
      True  : 정상 정지 (목표 도달 또는 물체 접촉으로 정지)
      False : 모터 에러 / 타임아웃 / 통신 불가

    target_pos를 주면 정지 위치가 목표와 GRIPPER_POS_TOLERANCE 이상 차이날 때
    경고를 출력한다 (물체 접촉 또는 스트로크 한계 가능성).
    """
    start = time.time()
    stable = 0
    prev_pos = None
    last = None
    motion_seen = False

    print("   ⏳ [그리퍼] 정지 확인 대기", end="", flush=True)
    while True:
        info = query_gripper_status(ser, debug=debug)
        elapsed = time.time() - start

        if info is None:
            print(" ⚠", end="", flush=True)
            stable = 0
        else:
            last = info
            status = info["status"]
            vel = info["velocity"]
            pos = info["finger_pos"]

            # --- 모터 에러(bit9) 즉시 중단 ---
            if status & (1 << 9):
                print(f"\n   🛑 [그리퍼] 모터 에러 감지! (status=0b{status:016b})")
                return False

            # --- 구형 응답(레지스터 5개 미수신) 방어: 정지 판정 불가 ---
            if vel is None or pos is None:
                print(" ?", end="", flush=True)
                stable = 0
            else:
                if vel != 0:
                    motion_seen = True

                pos_held = (prev_pos is not None and abs(pos - prev_pos) <= GRIPPER_POS_EPSILON)
                if vel == 0 and pos_held:
                    stable += 1
                else:
                    stable = 0
                prev_pos = pos

                # --- 조기 완료 방지 ---
                # 모터가 아직 한 번도 안 움직였다면, 아래 중 하나가 성립해야만
                # "정지"로 인정한다. 그렇지 않으면 출발을 더 기다린다.
                #   (a) 이미 목표 위치에 있음 -> 움직일 필요가 없는 정상 상황
                #   (b) GRIPPER_MOTION_GRACE 경과 -> 정말 안 움직이는 상황
                at_target = (target_pos is not None
                             and abs(pos - target_pos) <= GRIPPER_POS_TOLERANCE)
                ready = motion_seen or at_target or elapsed >= GRIPPER_MOTION_GRACE

                if stable >= settle_count and elapsed >= GRIPPER_MIN_WAIT and ready:
                    moved_txt = "" if motion_seen else " (이동 없음)"
                    print(f" [완료! finger={pos}, vel=0{moved_txt}]")

                    if target_pos is not None and abs(pos - target_pos) > GRIPPER_POS_TOLERANCE:
                        print(f"   ⚠️ [그리퍼] 목표({target_pos})와 실제({pos}) 차이 "
                              f"{abs(pos - target_pos)} - 물체 접촉 또는 스트로크 한계 가능성")
                    return True

                print(".", end="", flush=True)

        if elapsed > timeout:
            if last is not None:
                print(f"\n   ⏰ [그리퍼] 정지 확인 타임아웃! "
                      f"(status=0b{last['status']:016b}, vel={last['velocity']}, "
                      f"finger={last['finger_pos']}) -> 다음 단계로 진행")
            else:
                print("\n   ⏰ [그리퍼] 정지 확인 타임아웃! (응답 없음) -> 다음 단계로 진행")
            return False

        time.sleep(poll_interval)


def is_async_pipette_cmd(cmd_str):
    """Ld처럼 완료를 기다리면 안 되는 비동기(arm) 명령인지 판별"""
    return bool(cmd_str) and cmd_str.strip().startswith(PIPETTE_ASYNC_PREFIXES)


def send_pipette_oem(ser, addr, cmd_str):
    """
    피펫 OEM RS485 패킷 전송 및 응답 확인.
    반환: 접수 응답의 status 바이트(int). 응답이 없거나 파싱 실패 시 None.
    """
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

            # 응답이 올 때까지만 기다린다 (무조건 sleep 하지 않음)
            _t0 = time.time()
            while time.time() - _t0 < PIPETTE_ACK_TIMEOUT:
                if ser.in_waiting >= 4:      # 최소 프레임 길이
                    time.sleep(0.01)         # 나머지 바이트 도착 여유
                    break
                time.sleep(0.002)

            ack_status = None
            if ser.in_waiting > 0:
                resp = ser.read_all()
                print(f"      └ 📩 [응답 (Addr:{addr})]: {resp.hex(' ')}")
                # 응답 프레임: [0x55][addr][status][len][data...][checksum]
                if len(resp) >= 4 and resp[0] == 0x55 and resp[1] == addr:
                    ack_status = resp[2]
            else:
                print(f"      ⚠️ [응답 (Addr:{addr})]: 수신 응답 없음")

            time.sleep(PIPETTE_CMD_TAIL_DELAY)
            return ack_status
    except Exception as e:
        print(f"❌ 피펫 통신 에러: {e}")
    return None


def query_pipette_status(ser, addr):
    """
    OEM 프로토콜 '?' 상태 조회 명령 전송 후 status 코드 반환.
    응답 프레임: [0x55][addr][status][len][data...][checksum]
    status: 0=Idle, 1=Busy, 2=명령 성공 실행,
            10~19=파라미터 에러, 20~49=경고(계속 진행 가능), 50+=고장
    """
    try:
        if not (ser and ser.is_open):
            return None
        ser.reset_input_buffer()

        cmd_bytes = b'?'
        frame = bytearray([0xAA, addr, len(cmd_bytes)]) + cmd_bytes
        checksum = sum(frame) % 256
        frame.append(checksum)

        ser.write(frame)
        ser.flush()

        # --- 논블로킹 수신 ---
        # ser.read(64)를 쓰면 64바이트가 찰 때까지 시리얼 timeout(1초)을 통째로
        # 소진한다. 실제 응답은 5바이트뿐이라 폴링 1회당 1초씩 낭비됐다.
        resp = b''
        _t0 = time.time()
        while time.time() - _t0 < PIPETTE_STATUS_TIMEOUT:
            if ser.in_waiting >= 4:          # 최소 프레임 길이
                time.sleep(0.005)            # 나머지 바이트 도착 여유
                resp = ser.read(ser.in_waiting)
                break
            time.sleep(0.002)

        if len(resp) >= 4 and resp[0] == 0x55 and resp[1] == addr:
            return resp[2]
        return None
    except Exception as e:
        print(f"❌ [피펫 Addr:{addr}] 상태 조회 에러: {e}")
        return None


def wait_for_pipette_stop(ser, addr, timeout=PIPETTE_HANDSHAKE_TIMEOUT,
                          poll_interval=PIPETTE_POLL_INTERVAL, label=""):
    """
    피펫(SADP20) / 피펫 Z축이 완전히 정지(Idle=0 또는 성공=2)할 때까지 대기.
    10~19: 파라미터 에러 -> False 반환
    20~49: 경고(흡인/분사는 계속 가능) -> True 반환 (경고 로그만 남김)
    50+  : 고장 -> False 반환
    """
    start = time.time()
    print(f"   ⏳ [피펫{label} Addr:{addr}] 정지 확인 대기", end="", flush=True)
    while True:
        status = query_pipette_status(ser, addr)

        if status is None:
            print(" ⚠", end="", flush=True)

        # --- 0~9: 정상 작동 상태 ---
        elif status in (PIP_STATUS_IDLE, PIP_STATUS_SUCCESS):
            print(f" [완료! status={status}]")
            return True
        elif status == PIP_STATUS_LIQUID_FOUND:
            # 4 = 액체 감지됨. 고장이 아니라 정상 작동 상태다.
            print(f" [완료! status={status} 💧액체 감지됨]")
            return True
        elif status == PIP_STATUS_BUSY:
            print(".", end="", flush=True)
        elif 0 <= status <= 9:
            # 3, 5~9: 기타 작동 중 상태 -> 계속 대기
            print(f"({status})", end="", flush=True)

        # --- 10~19: 명령 실행 오류 ---
        elif 10 <= status <= 19:
            print(f"\n   ❌ [피펫{label} Addr:{addr}] 명령 실행 오류 (status={status})")
            return False

        # --- 20~49: 경고 (흡입/분사는 계속 가능) ---
        elif 20 <= status <= 49:
            if status == PIP_STATUS_TIMEOUT:
                print(f"\n   ⚠️ [피펫{label} Addr:{addr}] 타임아웃 (status=22) "
                      f"- 압력 변화 미감지")
            else:
                print(f"\n   ⚠️ [피펫{label} Addr:{addr}] 경고 상태 (status={status}) - 진행 가능")
            return True

        # --- 50 이상: 고장 ---
        else:
            print(f"\n   🛑 [피펫{label} Addr:{addr}] 장비 고장 (status={status})")
            return False

        if time.time() - start > timeout:
            print(f"\n   ⏰ [피펫{label} Addr:{addr}] 정지 확인 타임아웃! -> 다음 단계로 진행")
            return False

        time.sleep(poll_interval)


def read_pipz_register(ser, addr, reg_addr):
    """
    피펫 Z축 레지스터 읽기 (Rr 명령).
    자주 쓰는 레지스터:
      101 : 현재 위치 (um)      <- Zg 스톨 지점 확인용
      104 : 최대 스트로크 (mm)
      105 : 최대 속도 (mm/s)
      103 : 액체감지 비상정지 모드 (0=GPIO, 1=GPIO+통신)
    반환: 정수값, 실패 시 None
    """
    try:
        if not (ser and ser.is_open):
            return None
        ser.reset_input_buffer()

        cmd_bytes = f"Rr{reg_addr}".encode('ascii')
        frame = bytearray([0xAA, addr, len(cmd_bytes)]) + cmd_bytes
        frame.append(sum(frame) % 256)
        ser.write(frame)
        ser.flush()

        _t0 = time.time()
        while time.time() - _t0 < 0.3:
            if ser.in_waiting >= 4:
                time.sleep(0.01)
                break
            time.sleep(0.002)

        resp = ser.read_all()
        # [0x55][addr][status][len][ASCII 데이터...][checksum]
        if len(resp) >= 5 and resp[0] == 0x55 and resp[1] == addr:
            data_len = resp[3]
            if data_len > 0 and len(resp) >= 4 + data_len:
                text = resp[4:4 + data_len].decode('ascii', errors='ignore').strip()
                first = text.split(',')[0].strip()
                if first:
                    return int(first)
        return None
    except Exception as e:
        print(f"❌ [피펫 Z축 Addr:{addr}] 레지스터 읽기 에러: {e}")
        return None


def log_pipz_position(ser, addr, note=""):
    """
    피펫 Z축의 현재 위치를 읽어 출력한다.
    Zg(팁 장착) 직후에 보면 실제 스톨 지점을 알 수 있고,
    그 값을 기준으로 앞선 Zp 접근 높이를 조정하면 하강 거리를 줄일 수 있다.
    """
    pos = read_pipz_register(ser, addr, 101)
    if pos is None:
        return None
    print(f"      └ 📍 [피펫 Z축] 현재 위치: {pos} um ({pos/1000.0:.1f} mm){note}")
    return pos


def check_liquid_detected(ser, addr, label=" 본체"):
    """
    Ld(액체 감지)를 걸어둔 뒤 Z축 하강이 끝났을 때 감지 결과를 확인한다.
    반환: True=감지됨, False=미감지/타임아웃, None=판별 불가
    """
    status = query_pipette_status(ser, addr)

    if status is None:
        print(f"   ⚠️ [액체 감지] 상태 응답 없음 - 감지 여부 확인 불가")
        return None

    if status == PIP_STATUS_LIQUID_FOUND:
        print(f"   💧 [액체 감지] 감지 성공! (status={status})")
        return True

    if status == PIP_STATUS_TIMEOUT:
        print(f"   ⚠️ [액체 감지] 타임아웃 - 액체를 감지하지 못했습니다 (status={status})")
        print(f"      └ 확인 사항: 하강 깊이 부족 / 하강 속도 과다(권장 30mm/s 미만) / "
              f"팁 끝 이물질 / 감지 계수 설정")
        return False

    if status == PIP_STATUS_BUSY:
        print(f"   ⏳ [액체 감지] 아직 감지 대기 중 (status={status}) - 미감지 상태로 진행")
        return False

    print(f"   ℹ️ [액체 감지] 결과 미확정 (status={status})")
    return None


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

def wait_for_hotplate(ser, target_temp, boost_offset=10.0, timeout=1800, check_interval=2.0):
    """급속 예열 적용 및 자동 도달 확인 함수"""
    boost_temp = target_temp + boost_offset
    print(f"\n🚀 [핫플레이트] 급속 예열 동작 개시!")
    print(f"   ├ 목표 온도: {target_temp:.1f}°C")
    
    set_packet = set_temperature(boost_temp)
    ser.write(set_packet.encode('ascii'))
    time.sleep(0.2)
    
    start_time = time.time()
    
    while True:
        curr_temp = get_current_temperature(ser)
        
        if curr_temp is not None:
            print(f"   ⏳ [급속 예열 중...] 실제 온도: {curr_temp:.1f}°C / 목표: {target_temp:.1f}°C")
            
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
# 3. FMC4030 소켓 통신 헬퍼 함수
# ==========================================
def send_fmc_command(command_dict):
    """FMC4030 모션 컨트롤러 명령 전송 함수"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        client.connect((FMC_HOST, FMC_PORT))
        client.sendall(json.dumps(command_dict).encode('utf-8'))
        response_data = client.recv(1024).decode('utf-8')
        return json.loads(response_data)

def wait_for_fmc_stop(axis_num, timeout=60.0):
    """
    FMC4030 축 정지 대기.

    [이전 방식의 문제]
      이동 명령 직후 무조건 0.3초를 쉬고, 0.2초 간격으로 폴링하고,
      끝나고 다시 0.2초를 쉬었다. 축 이동 1회당 최소 0.5초가 고정으로 낭비되며
      레시피 전체(약 68회 이동)에서 34초 이상이 그냥 사라진다.

    [지금 방식]
      1) 빠르게(FMC_POLL_INTERVAL) 폴링한다.
      2) 축이 실제로 움직이기 시작(is_stop == 0)한 것을 확인하면,
         이후 정지 신호를 연속 FMC_STOP_CONFIRM회 받았을 때 완료로 인정한다.
      3) 처음부터 계속 정지 상태라면(= 이미 목표 위치라 움직일 필요가 없거나
         아직 출발 전) FMC_MOTION_GRACE 동안 출발을 기다린 뒤 완료 처리한다.
      -> 실제로 움직이는 이동은 오버헤드가 0.5초에서 0.06초 수준으로 줄고,
         움직일 필요 없는 이동도 0.5초 -> 0.2초로 줄어든다.
    """
    start = time.time()
    motion_seen = False
    stop_count = 0
    last_pos = 0.0

    print("   ⏳ 구동 중", end="", flush=True)
    while True:
        try:
            resp = send_fmc_command({"action": "check_stop", "axis": axis_num})
            is_stop = resp.get("is_stop")
            last_pos = resp.get("current_pos", 0.0)
            elapsed = time.time() - start

            if is_stop == 1:
                stop_count += 1
                # 움직인 적이 있으면 연속 확인만으로 완료.
                # 한 번도 안 움직였으면 출발을 조금 더 기다려본다.
                if motion_seen and stop_count >= FMC_STOP_CONFIRM:
                    print(f" [완료! (현재 위치: {last_pos:.2f} mm)]")
                    break
                if (not motion_seen) and elapsed >= FMC_MOTION_GRACE:
                    print(f" [완료! (이동 없음, 현재 위치: {last_pos:.2f} mm)]")
                    break
            else:
                motion_seen = True
                stop_count = 0
                print(".", end="", flush=True)

            if elapsed > timeout:
                print(f"\n   ⏰ [FMC] 축 {axis_num} 정지 확인 타임아웃!")
                break

            time.sleep(FMC_POLL_INTERVAL)

        except Exception as e:
            print(f"\n❌ 상태 확인 에러: {e}")
            break

    if FMC_TAIL_DELAY > 0:
        time.sleep(FMC_TAIL_DELAY)

def home_fmc_axis(axis_name, axis_num, speed=10.0, fall_step=5.0, direction=2):
    """FMC4030 축 원점 복귀 함수"""
    print(f"\n▶ [{axis_name}축] FMC4030 원점 호밍 시작...")
    command = {
        "action": "home", "axis": axis_num, "speed": speed,
        "acc_dec": 100.0, "fall_step": fall_step, "direction": direction
    }
    response = send_fmc_command(command)
    if response.get("res_code") == 0:
        wait_for_fmc_stop(axis_num)
        print(f"✅ [{axis_name}축] 원점 잡기 완료!")

def move_fmc_absolute(axis_name, axis_num, target_pos, speed=20.0):
    """FMC4030 축 절대 좌표 이동 함수"""
    print(f"   ├ 📐 [{axis_name}축] 절대 이동 -> 목표 좌표: {target_pos} mm (속도: {speed})")
    command = {
        "action": "move", "axis": axis_num,
        "pos": target_pos, "speed": speed, "mode": 2
    }
    response = send_fmc_command(command)
    if response.get("res_code") == 0:
        wait_for_fmc_stop(axis_num)

# ==========================================
# 4-0. ODrive 연결 관리 / 워치독 / keepalive
# ==========================================
class SpinCoaterFailure(Exception):
    """스핀코터 통신이 복구 불가능할 때 발생. 레시피를 안전하게 중단시킨다."""
    pass


def _is_odrive_comm_error(e):
    """
    ODrive USB 링크가 끊겼을 때 나는 예외인지 판별.
    라이브러리 버전마다 예외 클래스 이름이 달라 이름 기반으로 판별한다.
    (TransportException, ObjectLostError, DeviceLostException 등)
    """
    name = type(e).__name__
    return ("Transport" in name or "Lost" in name
            or "ChannelBroken" in name or "Disconnected" in name)


def connect_odrive(timeout=ODRIVE_FIND_TIMEOUT, verbose=True, calibrate=True):
    """
    ODrive를 탐색해 연결하고 엔코더 준비까지 마친 뒤 핸들을 반환한다.
    실패 시 None. 전역 ODRV도 함께 갱신한다.
    """
    global ODRV
    try:
        if verbose:
            print("🔍 ODrive 스핀코터 연결 중...")
        odrv = odrive.find_any(timeout=timeout)

        if hasattr(odrv, 'clear_errors'):
            odrv.clear_errors()

        # 인덱스 서치 비활성화는 연결 시 1회만 설정 (매 구동마다 쓰면 불필요한
        # config write가 발생해 USB 링크 부하와 실패 확률이 올라간다)
        try:
            odrv.axis0.encoder.config.use_index = False
        except Exception:
            pass

        if calibrate and not odrv.axis0.encoder.is_ready:
            if verbose:
                print("⚙️ [ODrive] 엔코더 영점 미설치 감지 -> 자동 캘리브레이션 진행 중...")
            odrv.axis0.encoder.config.calib_range = 0.5
            odrv.axis0.requested_state = 7

            calib_start = time.time()
            while odrv.axis0.current_state != 1:
                if time.time() - calib_start > ODRIVE_CALIB_TIMEOUT:
                    print("❌ [ODrive] 캘리브레이션 타임아웃!")
                    break
                time.sleep(0.1)
            time.sleep(0.5)

        if odrv.axis0.encoder.is_ready:
            if verbose:
                print("✅ ODrive 스핀코터 자동 준비 완료!")
        elif verbose:
            print("❌ ODrive 엔코더 정렬 실패!")

        ODRV = odrv
        return odrv

    except Exception as e:
        print(f"⚠️ ODrive 연결 실패: {e}")
        ODRV = None
        return None


def odrive_ping(odrv):
    """가벼운 읽기 한 번으로 USB 링크 생존 여부 확인"""
    if odrv is None:
        return False
    try:
        _ = odrv.vbus_voltage
        return True
    except Exception:
        return False


def ensure_odrive(verbose=True):
    """
    링크가 살아 있으면 현재 핸들을 그대로, 끊겼으면 재연결해 새 핸들을 반환.
    실패 시 None.
    """
    global ODRV
    with ODRV_LOCK:
        if odrive_ping(ODRV):
            return ODRV
        if verbose:
            print("\n   🔌 [ODrive] USB 링크 끊김 감지 -> 재연결 시도 중...")
        odrv = connect_odrive(timeout=ODRIVE_RECONNECT_TIMEOUT,
                              verbose=verbose, calibrate=False)
        if odrv is not None and verbose:
            print("   ✅ [ODrive] 재연결 성공!")
        return odrv


# ---------- 하드웨어 워치독 ----------
def odrive_watchdog_enable(odrv):
    """
    스핀 시작 시 호출. 통신이 ODRIVE_WATCHDOG_TIMEOUT 이상 끊기면
    ODrive가 스스로 축을 IDLE로 내려 모터를 멈춘다(폭주 방지).
    """
    global _odrv_watchdog_active
    if not ODRIVE_USE_WATCHDOG or odrv is None:
        return False
    try:
        odrv.axis0.config.watchdog_timeout = ODRIVE_WATCHDOG_TIMEOUT
        odrv.axis0.watchdog_feed()
        odrv.axis0.config.enable_watchdog = True
        _odrv_watchdog_active = True
        print(f"   🛡️ [스핀코터] 워치독 활성화 ({ODRIVE_WATCHDOG_TIMEOUT}초) "
              f"- 통신 두절 시 모터 자동 정지")
        return True
    except Exception as e:
        _odrv_watchdog_active = False
        print(f"   ⚠️ [스핀코터] 워치독을 지원하지 않는 펌웨어입니다 ({e}) - 미사용")
        return False


def odrive_watchdog_disable(odrv):
    """스핀 정지 후 호출. 유휴 구간에서 불필요하게 트립되지 않도록 끈다."""
    global _odrv_watchdog_active
    _odrv_watchdog_active = False
    if odrv is None:
        return
    try:
        odrv.axis0.config.enable_watchdog = False
    except Exception:
        pass


# ---------- 백그라운드 keepalive ----------
def _odrive_keepalive_worker():
    """
    메인 스레드가 FMC 이동이나 피펫 대기로 오래 블로킹돼 있어도
    ODrive 링크를 계속 살아 있게 유지하고 워치독을 먹인다.
    """
    global ODRV, _odrv_link_lost_count
    while not _odrv_keepalive_stop.wait(ODRIVE_KEEPALIVE_INTERVAL):
        with ODRV_LOCK:
            odrv = ODRV
            if odrv is None:
                continue
            try:
                _ = odrv.vbus_voltage
                if _odrv_watchdog_active:
                    try:
                        odrv.axis0.watchdog_feed()
                    except Exception:
                        pass
            except Exception as e:
                if not _is_odrive_comm_error(e):
                    continue
                _odrv_link_lost_count += 1
                print(f"\n   🔌 [ODrive/keepalive] 링크 두절 감지 "
                      f"(누적 {_odrv_link_lost_count}회) -> 재연결 시도")
                ODRV = None
                recovered = connect_odrive(timeout=ODRIVE_RECONNECT_TIMEOUT,
                                           verbose=False, calibrate=False)
                if recovered is not None:
                    print("   ✅ [ODrive/keepalive] 재연결 성공")
                    if _odrv_watchdog_active:
                        odrive_watchdog_enable(recovered)
                else:
                    print("   ❌ [ODrive/keepalive] 재연결 실패 - 다음 주기에 재시도")


def start_odrive_keepalive():
    global _odrv_keepalive_thread
    if _odrv_keepalive_thread is not None:
        return
    _odrv_keepalive_stop.clear()
    _odrv_keepalive_thread = threading.Thread(
        target=_odrive_keepalive_worker, name="odrv-keepalive", daemon=True)
    _odrv_keepalive_thread.start()
    print(f"🫀 [ODrive] keepalive 시작 ({ODRIVE_KEEPALIVE_INTERVAL}초 주기)")


def stop_odrive_keepalive():
    global _odrv_keepalive_thread
    _odrv_keepalive_stop.set()
    if _odrv_keepalive_thread is not None:
        _odrv_keepalive_thread.join(timeout=2.0)
        _odrv_keepalive_thread = None


# ---------- 공정 시작 전 링크 안정성 시험 ----------
def odrive_preflight_check(odrv, duration=ODRIVE_PREFLIGHT_DURATION):
    """
    공정을 시작하기 전에 USB 링크를 집중적으로 두드려 안정성을 확인한다.
    반환: (성공횟수, 실패횟수)
    """
    if odrv is None:
        return (0, 1)

    print(f"\n🧪 [ODrive] 링크 안정성 사전 점검 ({duration:.0f}초)...")
    ok = fail = 0
    start = time.time()
    while time.time() - start < duration:
        with ODRV_LOCK:
            try:
                _ = ODRV.vbus_voltage if ODRV is not None else odrv.vbus_voltage
                ok += 1
            except Exception:
                fail += 1
        time.sleep(0.01)

    total = ok + fail
    rate = (fail / total * 100.0) if total else 100.0
    print(f"   └ 결과: {ok}회 성공 / {fail}회 실패 (실패율 {rate:.2f}%)")
    return (ok, fail)


# ==========================================
# 4. ODrive 스핀코터 제어 함수 (안전 1초 감속 적용)
# ==========================================
def control_spin_cooter(odrv, target_rpm, accel_time_sec=1.0, recipe_start_time=None):
    """
    스핀코터 제어 진입점.
    USB 링크가 끊겨 통신 예외가 나면 재연결 후 자동 재시도한다.
    끝내 복구되지 않으면 SpinCoaterFailure를 던져 레시피를 안전하게 중단시킨다.
    """
    global ODRV
    if ODRV is None and odrv is not None:
        ODRV = odrv

    last_err = None
    for attempt in range(1, ODRIVE_MAX_RETRY + 1):
        target = ODRV if attempt == 1 else ensure_odrive()
        if target is None:
            last_err = "ODrive 핸들 없음"
            time.sleep(1.0)
            continue

        try:
            with ODRV_LOCK:
                result = _control_spin_cooter_body(
                    target, target_rpm,
                    accel_time_sec=accel_time_sec,
                    recipe_start_time=recipe_start_time
                )
            return result

        except Exception as e:
            if not _is_odrive_comm_error(e):
                raise
            last_err = f"{type(e).__name__}: {e}"
            print(f"\n   ⚠️ [스핀코터] USB 통신 오류 ({last_err})")
            ODRV = None
            if attempt < ODRIVE_MAX_RETRY:
                print(f"   🔁 [스핀코터] 재연결 후 재시도 "
                      f"{attempt}/{ODRIVE_MAX_RETRY - 1}...")
                time.sleep(1.0)

    # --- 복구 실패: 여기서 멈추는 것이 안전하다 ---
    print(f"\n   ❌ [스핀코터] {ODRIVE_MAX_RETRY}회 시도 후에도 통신 복구 실패")
    if target_rpm > 0:
        raise SpinCoaterFailure(
            f"스핀 기동 실패로 공정 중단 ({last_err}). 시료가 오염되지 않도록 정지합니다.")
    else:
        # 정지 명령이 실패한 경우 -> 모터가 계속 돌고 있을 수 있다. 최대 경고.
        print("   🚨🚨 [경고] 스핀 정지 명령 전달 실패! 모터가 회전 중일 수 있습니다.")
        if ODRIVE_USE_WATCHDOG and _odrv_watchdog_active:
            print(f"   🛡️ 워치독이 활성화되어 있어 {ODRIVE_WATCHDOG_TIMEOUT}초 내로 "
                  f"모터가 자동 정지합니다.")
        else:
            print("   🚨 워치독 미활성 상태입니다. 수동으로 전원을 차단하세요!")
        raise SpinCoaterFailure(f"스핀 정지 실패 ({last_err}). 즉시 확인이 필요합니다.")


def verify_spin_speed(odrv, target_rpm, settle=1.5,
                      tolerance=ODRIVE_SPIN_VERIFY_TOLERANCE):
    """
    스핀 기동 후 실제 회전수가 목표에 도달했는지 확인.
    도달하지 못하면 False (시료를 버리기 전에 잡아내기 위함).
    """
    if not ODRIVE_VERIFY_SPIN or odrv is None or target_rpm <= 0:
        return True
    time.sleep(settle)
    try:
        with ODRV_LOCK:
            actual = abs(odrv.axis0.encoder.vel_estimate) * 60.0
    except Exception as e:
        print(f"   ⚠️ [스핀코터] 속도 검증 실패(통신): {e}")
        return False

    lo = target_rpm * (1.0 - tolerance)
    if actual < lo:
        print(f"   ❌ [스핀코터] 목표 {target_rpm} RPM 대비 실제 {actual:.0f} RPM "
              f"- 기동 이상!")
        return False
    print(f"   ✅ [스핀코터] 회전 확인: {actual:.0f} RPM (목표 {target_rpm})")
    return True


def _control_spin_cooter_body(odrv, target_rpm, accel_time_sec=1.0, recipe_start_time=None):
    global odrv_start_pos

    if target_rpm > 0:
        target_rps = target_rpm / 60.0
        if accel_time_sec <= 0:
            accel_time_sec = 0.1

        try:
            current_rps = odrv.axis0.controller.input_vel
        except Exception:
            current_rps = 0.0

        delta_rps = abs(target_rps - current_rps)
        ramp_rate_rps = delta_rps / accel_time_sec if delta_rps > 0 else (target_rps / accel_time_sec)

        print(f"\n🌀 [스핀코터] {target_rpm} RPM 구동/변속 명령 (목표 가속 시간: {accel_time_sec}초)")
        print(f"   └ 📐 계산된 가속도(Ramp Rate): {ramp_rate_rps:.2f} RPS/s")

        # use_index는 connect_odrive()에서 1회 설정한다 (매 구동마다 config write를
        # 하면 USB 왕복이 늘어 노이즈 환경에서 실패 확률이 올라간다)
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
        
        odrv.axis0.controller.config.vel_ramp_rate = ramp_rate_rps
        odrv.axis0.controller.config.vel_gain = 0.05
        odrv.axis0.controller.config.vel_integrator_gain = 0.2

        if odrv.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            odrv.axis0.controller.input_vel = 0.0
            odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.1)

        odrv.axis0.controller.input_vel = target_rps
        print(f"   🚀 [스핀코터] {target_rpm} RPM으로 회전 가동 중")

        # 회전이 시작된 시점부터 워치독을 켠다.
        # 이후 통신이 끊기면 ODrive가 스스로 모터를 정지시킨다.
        odrive_watchdog_enable(odrv)

    else:
        print(f"\n⏹️ [스핀코터] 정지 명령 수신 -> 안전 1초 감속 및 정밀 영점 안착 시작...")
        
        # 💡 안전 1초 감속을 위한 Ramp Rate 자동 지정
        try:
            current_rps = abs(odrv.axis0.encoder.vel_estimate)
        except Exception:
            current_rps = 0.0

        decel_time_sec = 1.0  # 1초 감속 목표
        ramp_rate_rps = current_rps / decel_time_sec if current_rps > 0 else 83.3

        odrv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        odrv.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        odrv.axis0.controller.config.vel_ramp_rate = max(ramp_rate_rps, 10.0)
        odrv.axis0.controller.input_vel = 0.0

        while abs(odrv.axis0.encoder.vel_estimate) > 0.5:
            proc_time = time.time() - recipe_start_time if recipe_start_time is not None else 0.0
            curr_rpm = odrv.axis0.encoder.vel_estimate * 60.0
            print(f"\r   🛑 [공정 시간: {proc_time:6.1f}초] 1초 감속 중... 현재 속도: {curr_rpm:6.1f} RPM", end="", flush=True)
            time.sleep(0.02)
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
        odrive_watchdog_disable(odrv)

    return True

# ==========================================
# 5. 통합 프로세스 레시피
# ==========================================
PROCESS_RECIPE = [

    # 초기화[cite: 8]
    {"step": "초기화 확인 및 펌프 대기", "x": None, "y": None, "z": None, "grip": 1000, "pip_z": None, "pipette": "It16000,100,2", "hotplate": 0, "pump": None, "spin": None, "delay": 0.1},
   
      
    #cell plate 1-1
    {"step": "이동", "x": 512.5, "y": None, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay": 0.1},
    {"step": "이동", "x": None, "y": 26, "z":None, "grip": None, "pip_z": None, "pipette": None, "hotplate": None, "pump": None, "spin": None, "delay":  0.1},
    {"step": "그리퍼", "x": None, "y": None, "z": None, "grip": 650, "delay": 0.1},
    {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 110.2, "grip": None, "delay": 0.1},
    {"step": "그리퍼", "x": None, "y": None, "z": None, "grip": 520, "delay": 0.1},
    {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 10, "grip": None, "delay": 0.1},   

    #hot plate 1-1
    {"step": "X/Y/Z축 이동 동작", "x": 13.6, "y": None, "z": None, "grip": None, "delay": 0.1},
    {"step": "X/Y/Z축 이동 동작", "x": None, "y": 21.9, "z": None, "grip": None, "delay": 0.1},
    {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 42.3, "grip": None, "delay": 0.1},
    {"step": "그리퍼", "x": None, "y": None, "z": None, "grip": 675, "delay": 0.1},
    {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 10, "grip": None, "delay": 0.1},   
    {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 42.3, "grip": None, "delay": 0.1},
    {"step": "그리퍼", "x": None, "y": None, "z": None, "grip": 520, "delay": 0.1},
    {"step": "X/Y/Z축 이동 동작", "x": None, "y": None, "z": 10, "grip": None, "delay": 0.1},   
           


]

# ==========================================
# 6. 메인 자동화 실행 루프
# ==========================================
if __name__ == "__main__":
    print("=== 🤖 FULL PASCAL 자동화 통합 제어 시스템 가동 ===")
    
    s_grip = None
    s_pip = None
    s_hotplate = None
    s_relay = None
    odrv0 = None

    # 1. 하드웨어 시리얼 및 ODrive 연결[cite: 8]
    try:
        s_grip = serial.Serial(GRIPPER_PORT, BAUD_GRIP, timeout=1)
        s_pip  = serial.Serial(PIPETTE_PORT, BAUD_PIP, timeout=1)
        
        s_hotplate = serial.Serial(
            HOTPLATE_PORT, 
            baudrate=9600, 
            bytesize=serial.SEVENBITS, 
            parity=serial.PARITY_EVEN, 
            stopbits=serial.STOPBITS_ONE, 
            timeout=2
        )
        
        s_relay = serial.Serial(RELAY_PORT, BAUD_RELAY, timeout=1)
        print("✅ 모든 RS-485 및 릴레이 시리얼 연결 완료!")
    except Exception as e:
        print(f"⚠️ 일부 시리얼 포트 연결 실패: {e}")

    odrv0 = connect_odrive()
    if odrv0 is not None:
        start_odrive_keepalive()

    # 초기 가드 설정[cite: 8]
    if s_relay and s_relay.is_open:
        s_relay.write(b"P0\n")
        s_relay.flush()
        time.sleep(0.05)

    # 2. 피펫 Z축 상단 대피 & FMC4030 원점 잡기[cite: 8]
    if s_pip and s_pip.is_open:
        print("\n🛡️ [안전] 피펫 Z축 최상단 안전 위치 대피...")
        send_pipette_oem(s_pip, P_Z_ADDR, "Zz50000")
        if HANDSHAKE_OK["pip_z"]:
            wait_for_pipette_stop(s_pip, P_Z_ADDR,
                                  timeout=PIPZ_HANDSHAKE_TIMEOUT, label=" Z축")
        else:
            time.sleep(3.0)

    home_fmc_axis("Z", axis_num=2, speed=25.0, fall_step=5.0, direction=2)
    home_fmc_axis("X", axis_num=0, speed=30.0, fall_step=5.0, direction=2)
    home_fmc_axis("Y", axis_num=1, speed=30.0, fall_step=5.0, direction=2)
    
    if s_grip and s_grip.is_open:
        # --- [진단] 그리퍼 상태 레지스터 원시 응답 확인 ---
        if GRIPPER_DEBUG and GRIPPER_DEBUG_PROBE_COUNT > 0:
            print("\n[TEST] 그리퍼 상태 레지스터 원시 응답 확인")
            for _ in range(GRIPPER_DEBUG_PROBE_COUNT):
                probe = query_gripper_status(s_grip, debug=True)
                if probe is None:
                    print("      -> 파싱 결과: (없음)")
                time.sleep(0.5)

        print("\n[INIT] 그리퍼 원점 초기화 탐색...")
        send_gripper_cmd(s_grip, 101, 0)
        if HANDSHAKE_OK["gripper"]:
            # 초기화는 전 스트로크를 훑는 큰 동작이므로 넉넉한 타임아웃과
            # 긴 최소 관찰시간(1초)을 적용. bit1(Initialize)도 함께 확인한다.
            init_start = time.time()
            init_stable = 0
            init_prev = None
            print("   ⏳ [그리퍼] 초기화 완료 대기", end="", flush=True)
            while time.time() - init_start < 20.0:
                r = query_gripper_status(s_grip, debug=False)
                if r is None:
                    print(" ⚠", end="", flush=True)
                    init_stable = 0
                else:
                    if r["status"] & (1 << 9):
                        print(f"\n   🛑 [그리퍼] 초기화 중 모터 에러! (status=0b{r['status']:016b})")
                        break

                    initializing = bool(r["status"] & (1 << 1))   # bit1
                    vel, pos = r["velocity"], r["finger_pos"]

                    if vel is None or pos is None:
                        init_stable = 0
                    else:
                        held = (init_prev is not None and abs(pos - init_prev) <= GRIPPER_POS_EPSILON)
                        init_stable = init_stable + 1 if (vel == 0 and held) else 0
                        init_prev = pos

                        if (not initializing) and init_stable >= GRIPPER_SETTLE_COUNT \
                                and (time.time() - init_start) >= 1.0:
                            print(f" [초기화 완료! finger={pos}, status=0b{r['status']:016b}]")
                            break
                    print(".", end="", flush=True)
                time.sleep(0.1)
            else:
                print("\n   ⏰ [그리퍼] 초기화 확인 타임아웃 -> 계속 진행")
        else:
            time.sleep(4.0)

    # 3. 핫플레이트 급속 예열 부스팅 단계[cite: 8]
    if s_hotplate and s_hotplate.is_open:
        wait_for_hotplate(
            ser=s_hotplate, 
            target_temp=TARGET_INIT_HOTPLATE_TEMP, 
            boost_offset=HOTPLATE_BOOST_OFFSET
        )
    else:
        print(f"\n⚠️ 핫플레이트 시리얼 미연결로 예열 로직을 생략합니다.")

    # --- ODrive 링크 안정성 사전 점검 ---
    # 여기서 걸러내면 시료를 버리지 않는다. 공정 중간 실패보다 훨씬 낫다.
    if ODRIVE_PREFLIGHT_ENABLE and odrv0 is not None:
        _ok, _fail = odrive_preflight_check(odrv0)
        if _fail > ODRIVE_PREFLIGHT_MAX_FAIL:
            print("\n🚨 [ODrive] 링크가 불안정합니다!")
            print("   └ USB 케이블/아이솔레이터/접지를 점검하세요.")
            if ODRIVE_PREFLIGHT_ABORT:
                stop_odrive_keepalive()
                raise SystemExit("❌ ODrive 링크 불안정 - 시료 손상을 막기 위해 "
                                 "공정을 시작하지 않습니다.")
            print("   ⚠️ 경고를 무시하고 공정을 진행합니다.")
        else:
            print("   ✅ 링크 안정성 양호 - 공정을 시작합니다.")

    print("✨ 모든 시스템 준비 완료! 통합 레시피를 시작합니다.\n")
    time.sleep(1)

    # 4. 레시피 실행[cite: 8]
    try:
        spin_start_time = None
        target_spin_duration = 0.0
        liquid_detect_armed = False   # Ld로 액체 감지를 켜둔 상태인지

        for idx, task in enumerate(PROCESS_RECIPE):
            print(f"\n▶ [STEP {idx+1}] {task['step']}")

            if task.get('wait_spin_time') is not None and spin_start_time is not None:
                target_wait = task['wait_spin_time']
                elapsed = time.time() - spin_start_time
                remaining = target_wait - elapsed

                if remaining > 0:
                    print(f"   ⏳ [정밀 타이밍] 스핀 회전 시작 후 {target_wait}초 시점까지 {remaining:.2f}초 추가 대기 중...")
                    time.sleep(remaining)

            # --- [1] FMC4030 XYZ 절대 좌표 이동 ---[cite: 8]
            if task.get('x') is not None:
                move_fmc_absolute("X", axis_num=0, target_pos=task['x'], speed=300.0)
                
            if task.get('y') is not None:
                move_fmc_absolute("Y", axis_num=1, target_pos=task['y'], speed=300.0)
                
            if task.get('z') is not None:
                move_fmc_absolute("Z", axis_num=2, target_pos=task['z'], speed=70.0)

            # --- [2] 그리퍼 ---[cite: 8]
            if task.get('grip') is not None and s_grip and s_grip.is_open:
                send_gripper_cmd(s_grip, 104, task['grip'])
                if HANDSHAKE_OK["gripper"]:
                    wait_for_gripper_stop(s_grip, target_pos=task['grip'], debug=GRIPPER_DEBUG)
                
            # --- [3] 피펫 Z축 (Addr: 41) ---[cite: 8]
            if task.get('pip_z') is not None and s_pip and s_pip.is_open:
                send_pipette_oem(s_pip, P_Z_ADDR, task['pip_z'])
                if HANDSHAKE_OK["pip_z"]:
                    wait_for_pipette_stop(s_pip, P_Z_ADDR,
                                          timeout=PIPZ_HANDSHAKE_TIMEOUT, label=" Z축")

                # Zg(팁 장착)는 스톨할 때까지 내려가므로, 실제 정지 지점을 기록해두면
                # 앞선 Zp 접근 높이를 얼마나 낮출 수 있는지 바로 알 수 있다.
                if PIPZ_LOG_POSITION:
                    _note = " ← Zg 스톨 지점" if str(task['pip_z']).startswith("Zg") else ""
                    log_pipz_position(s_pip, P_Z_ADDR, note=_note)

                # Ld를 걸어둔 상태였다면, 이번 하강에서 액체를 감지했는지 확인.
                # (감지되면 Z축은 레지스터 103 설정에 따라 자동으로 정지한다)
                if liquid_detect_armed:
                    check_liquid_detected(s_pip, P_ADDR)
                    liquid_detect_armed = False

            # --- [4] 피펫 본체 SADP20 (Addr: 1) ---[cite: 8]
            if task.get('pipette') is not None and s_pip and s_pip.is_open:
                pip_cmd = task['pipette']
                ack = send_pipette_oem(s_pip, P_ADDR, pip_cmd)

                if is_async_pipette_cmd(pip_cmd):
                    # Ld 같은 arm 명령: 완료를 기다리면 안 된다.
                    # 접수(status 2 = 실행 성공)만 확인하고 즉시 다음 스텝으로 넘어가서
                    # Z축이 하강하는 동안 백그라운드로 감지가 동작하게 한다.
                    if ack == PIP_STATUS_SUCCESS or ack == PIP_STATUS_IDLE:
                        print(f"   ▶ [피펫 본체] 비동기 명령 접수 완료 (ack={ack}) "
                              f"- 대기 없이 진행, 하강 중 백그라운드 감시")
                        liquid_detect_armed = True
                    elif ack is None:
                        print(f"   ⚠️ [피펫 본체] 비동기 명령 접수 응답 없음 - 감지 동작 불확실")
                        liquid_detect_armed = True
                    else:
                        print(f"   ❌ [피펫 본체] 비동기 명령 거부됨 (ack={ack}) - 감지 미동작")
                        liquid_detect_armed = False

                elif HANDSHAKE_OK["pipette"]:
                    wait_for_pipette_stop(s_pip, P_ADDR, label=" 본체")
                
            # --- [5] 핫플레이트 ---[cite: 8]
            if task.get('hotplate') is not None and s_hotplate and s_hotplate.is_open:
                packet = set_temperature(task['hotplate'])
                s_hotplate.write(packet.encode('ascii'))
                print(f"   └ ♨️ [핫플레이트] {task['hotplate']}°C 설정 전송")

            # --- [6] 진공 펌프 ---[cite: 8]
            if task.get('pump') is not None:
                pump_val = int(task['pump']) 
                if s_relay and s_relay.is_open:
                    cmd = f"P{pump_val}\n".encode('ascii')
                    s_relay.write(cmd)
                    s_relay.flush()  
                time.sleep(0.1)
                print(f"   └ 💨 [진공 펌프] {'ON (P1)' if pump_val == 1 else 'OFF (P0)'} 전송 완료!")
                
            # --- [7] ODrive 스핀코터 ON/OFF 제어 ---[cite: 8]
            if task.get('spin') is not None and odrv0 is not None:
                target_rpm = task['spin']
                accel_time_val = task.get('accel_time', 1.0)
                
                if target_rpm > 0:
                    spin_start_time = time.time()
                    target_spin_duration = task.get('spin_time', 45.0)
                    control_spin_cooter(odrv0, target_rpm,
                                        accel_time_sec=accel_time_val,
                                        recipe_start_time=spin_start_time)
                    odrv0 = ODRV if ODRV is not None else odrv0

                    # 실제로 목표 RPM에 도달했는지 확인 (기동 이상 조기 발견)
                    if not verify_spin_speed(odrv0, target_rpm,
                                             settle=max(accel_time_val, 1.0) + 0.5):
                        raise SpinCoaterFailure(
                            f"스핀 기동 검증 실패 (목표 {target_rpm} RPM 미달)")
                else:
                    if spin_start_time is not None:
                        elapsed = time.time() - spin_start_time
                        remaining = target_spin_duration - elapsed
                        if remaining > 0:
                            time.sleep(remaining)
                    control_spin_cooter(odrv0, 0, recipe_start_time=spin_start_time)
                    odrv0 = ODRV if ODRV is not None else odrv0
                    spin_start_time = None

            # --- [8] Step Delay ---[cite: 8]
            # 핸드셰이크가 실제 완료를 확인하므로 이 delay는 여유분이다.
            # STEP_DELAY_SCALE로 일괄 조정한다 (0.0이면 생략).
            delay_time = task.get('delay', 0.1) * STEP_DELAY_SCALE
            if delay_time > 0:
                time.sleep(delay_time)

        print("\n🎉 모든 레시피 공정이 성공적으로 완료되었습니다!")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단 (Ctrl+C)")
    except SpinCoaterFailure as e:
        print(f"\n🚨 [공정 중단] {e}")
        print("   └ 갠트리와 피펫을 더 이상 움직이지 않고 안전하게 종료합니다.")
    finally:
        # --- 스핀 모터부터 확실히 정지 ---
        stop_odrive_keepalive()
        _final_odrv = ODRV if ODRV is not None else odrv0
        if _final_odrv is not None:
            try:
                odrive_watchdog_disable(_final_odrv)
            except: pass
            try:
                _final_odrv.axis0.requested_state = AXIS_STATE_IDLE
                print("🌀 [스핀코터] 모터 IDLE 전환 완료")
            except Exception as e:
                print(f"🚨 [스핀코터] 정지 명령 전달 실패: {e}")
                print("   └ 모터가 회전 중일 수 있습니다. 직접 확인하세요!")

        if s_relay and s_relay.is_open:
            try:
                s_relay.write(b"P0\n")
                s_relay.flush()
                time.sleep(0.05)
                s_relay.close()
            except: pass

        if s_hotplate and s_hotplate.is_open:
            try:
                off_packet = set_temperature(0.0)
                s_hotplate.write(off_packet.encode('ascii'))
                s_hotplate.flush()
                time.sleep(0.1)
                s_hotplate.close()
                print("♨️ [핫플레이트] 안전 히터 OFF 완료")
            except: pass

        if s_grip and s_grip.is_open: s_grip.close()
        if s_pip and s_pip.is_open: s_pip.close()
        print("🔌 모든 하드웨어 포트가 안전하게 종료되었습니다.")