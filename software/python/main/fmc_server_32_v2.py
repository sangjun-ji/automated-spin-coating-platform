# =====================================================================
# fmc_server_32_v2.py
#
# 기존 fmc_server_32.py 대비 변경점
#   1) 가감속(acc/dec)을 명령에서 받는다. 미지정 시 기존과 동일한 300.0.
#   2) check_stop_fast 액션 추가 - 위치조회를 생략해 컨트롤러 왕복을 절반으로.
#   3) DLL 호출 시간을 서버에서 직접 계측. 200ms 멈춤이 어디서 나는지 판별용.
#   4) 종료 시 Close_Device 호출.
#      SDK: "프로그램 종료 전 반드시 호출해 리소스를 해제해야 하며,
#            그렇지 않으면 다음 연결이 실패한다."
#   5) 클라이언트 소켓 재사용 지원 - 한 연결에서 여러 명령을 처리한다.
#      기존 클라이언트(명령 1건당 연결)도 그대로 동작한다.
#
# 기존 파일은 지우지 말고 남겨둘 것. 문제 생기면 되돌린다.
# =====================================================================

import ctypes
import socket
import json
import time
import atexit

DLL_PATH = r"C:\Users\aned\OneDrive\Desktop\자율주행\코드\FMC4030-Dll.dll"

# --- 진단 설정 ---
PROFILE_DLL      = True    # DLL 호출 시간 계측
SLOW_CALL_MS     = 50.0    # 이 시간을 넘는 DLL 호출만 출력
STATS_EVERY      = 500     # N회마다 누적 통계 출력

try:
    fmc = ctypes.WinDLL(DLL_PATH)
except Exception as e:
    print(f"DLL 로드 실패: {e}")
    exit()

# ==========================================
# C언어 함수 규격 설정
# ==========================================
fmc_open       = getattr(fmc, "_FMC4030_Open_Device@12")
fmc_jog        = getattr(fmc, "_FMC4030_Jog_Single_Axis@28")
fmc_get_pos    = getattr(fmc, "_FMC4030_Get_Axis_Current_Pos@12")
fmc_check_stop = getattr(fmc, "_FMC4030_Check_Axis_Is_Stop@8")
fmc_home       = getattr(fmc, "_FMC4030_Home_Single_Axis@24")

fmc_open.argtypes = [ctypes.c_long, ctypes.c_char_p, ctypes.c_long]
fmc_open.restype = ctypes.c_long

fmc_jog.argtypes = [
    ctypes.c_long, ctypes.c_long, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_long
]
fmc_jog.restype = ctypes.c_long

fmc_get_pos.argtypes = [ctypes.c_long, ctypes.c_long, ctypes.POINTER(ctypes.c_float)]
fmc_get_pos.restype = ctypes.c_long

fmc_check_stop.argtypes = [ctypes.c_long, ctypes.c_long]
fmc_check_stop.restype = ctypes.c_long

fmc_home.argtypes = [
    ctypes.c_long, ctypes.c_long,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_long
]
fmc_home.restype = ctypes.c_long

# --- Close_Device (선택) ---
# stdcall 데코레이션 이름이 환경마다 다를 수 있어 몇 가지를 시도한다.
fmc_close = None
for _n in ("_FMC4030_Close_Device@4", "FMC4030_Close_Device"):
    try:
        fmc_close = getattr(fmc, _n)
        fmc_close.argtypes = [ctypes.c_long]
        fmc_close.restype = ctypes.c_long
        print(f"Close_Device 심볼 확인: {_n}")
        break
    except AttributeError:
        continue
if fmc_close is None:
    print("주의: Close_Device 심볼을 찾지 못했다. 종료 시 자원 해제가 안 된다.")


# ==========================================
# DLL 호출 계측 래퍼
# ==========================================
_stats = {}   # 함수명 -> [호출수, 합계ms, 최대ms, 느린호출수]


def timed(name, fn, *args):
    """DLL 호출 시간을 재고, 느린 호출을 표시한다."""
    if not PROFILE_DLL:
        return fn(*args)

    t = time.perf_counter()
    r = fn(*args)
    ms = (time.perf_counter() - t) * 1000.0

    s = _stats.setdefault(name, [0, 0.0, 0.0, 0])
    s[0] += 1
    s[1] += ms
    if ms > s[2]:
        s[2] = ms
    if ms >= SLOW_CALL_MS:
        s[3] += 1
        print(f"  [SLOW] {name} {ms:7.1f} ms   (누적 느린호출 {s[3]}/{s[0]})")

    if STATS_EVERY and s[0] % STATS_EVERY == 0:
        print(f"  [STAT] {name}: {s[0]}회  평균 {s[1]/s[0]:6.2f}ms  "
              f"최대 {s[2]:7.1f}ms  느린호출 {s[3]}회 "
              f"({s[3]/s[0]*100:.1f}%)")
    return r


# ==========================================
# 장비 연결
# ==========================================
device_id = 0
ip = b"192.168.0.30"
port = 8088

print("장비 연결을 시도합니다...")
res = fmc_open(device_id, ip, port)
if res < 0:
    print(f"장비 연결 실패 (에러 코드: {res})")
    exit()
print("기계 연결 성공!")

if fmc_close is not None:
    atexit.register(lambda: (print("Close_Device 호출"), fmc_close(device_id)))


# ==========================================
# 명령 처리
# ==========================================
def handle(cmd_data):
    action = cmd_data.get('action')

    # --- 절대/상대 이동 ---
    if action == 'move':
        axis  = cmd_data['axis']
        pos   = cmd_data['pos']
        speed = cmd_data['speed']
        mode  = cmd_data.get('mode', 2)      # 1: 상대, 2: 절대

        # 가감속. 미지정 시 기존 동작과 동일한 300.0.
        acc = float(cmd_data.get('acc', 300.0))
        dec = float(cmd_data.get('dec', acc))

        res_code = timed("jog", fmc_jog,
                         device_id, axis,
                         ctypes.c_float(pos), ctypes.c_float(speed),
                         ctypes.c_float(acc), ctypes.c_float(dec), mode)

        current_pos = ctypes.c_float(0.0)
        timed("get_pos", fmc_get_pos, device_id, axis, ctypes.byref(current_pos))

        return {"status": "OK", "res_code": res_code,
                "current_pos": current_pos.value, "acc": acc, "dec": dec}

    # --- 정지 확인 (위치 포함) ---
    elif action == 'check_stop':
        axis = cmd_data['axis']
        is_stop = timed("check_stop", fmc_check_stop, device_id, axis)

        current_pos = ctypes.c_float(0.0)
        timed("get_pos", fmc_get_pos, device_id, axis, ctypes.byref(current_pos))

        return {"is_stop": is_stop, "current_pos": current_pos.value}

    # --- 정지 확인 (경량, 폴링 전용) ---
    # 컨트롤러 왕복이 1회. 폴링 루프는 이것만 쓰고,
    # 완료 후 위치가 필요하면 check_stop 을 한 번만 호출한다.
    elif action == 'check_stop':
        axis = cmd_data['axis']
        is_stop = timed("check_stop", fmc_check_stop, device_id, axis)

        current_pos = ctypes.c_float(0.0)
        rc = timed("get_pos", fmc_get_pos, device_id, axis, ctypes.byref(current_pos))
        if rc != 0:
            print(f"  [ERR] get_pos axis{axis} 실패 rc={rc}")

        return {"is_stop": is_stop, "current_pos": current_pos.value, "pos_rc": rc}

    # --- 원점 호밍 ---
    elif action == 'home':
        axis      = cmd_data['axis']
        speed     = cmd_data.get('speed', 50.0)
        acc_dec   = cmd_data.get('acc_dec', 100.0)
        fall_step = cmd_data.get('fall_step', 5.0)
        direction = cmd_data.get('direction', 2)

        res_code = timed("home", fmc_home,
                         device_id, axis,
                         ctypes.c_float(speed), ctypes.c_float(acc_dec),
                         ctypes.c_float(fall_step), direction)
        return {"status": "OK", "res_code": res_code}

    # --- 누적 통계 조회 ---
    elif action == 'stats':
        return {"stats": {k: {"calls": v[0],
                              "avg_ms": round(v[1] / v[0], 2) if v[0] else 0,
                              "max_ms": round(v[2], 1),
                              "slow": v[3]} for k, v in _stats.items()}}

    return {"error": f"unknown action: {action}"}


# ==========================================
# 소켓 서버
# ==========================================
HOST = '127.0.0.1'
PORT = 50000
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(16)
print(f"32비트 서버 가동 완료. 명령 대기 중... (포트: {PORT})")
print(f"  가감속: 명령에 acc/dec 미지정 시 300.0 사용")
print(f"  DLL 계측: {'ON' if PROFILE_DLL else 'OFF'}  "
      f"(느린 호출 기준 {SLOW_CALL_MS:.0f}ms)")

while True:
    conn, addr = server.accept()
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(30.0)

        # 한 연결에서 여러 명령을 처리한다.
        # 명령 1건만 보내고 닫는 기존 클라이언트도 그대로 동작한다.
        while True:
            data = conn.recv(1024)
            if not data:
                break
            try:
                cmd_data = json.loads(data.decode('utf-8'))
            except json.JSONDecodeError:
                break
            resp = handle(cmd_data)
            conn.sendall(json.dumps(resp).encode('utf-8'))

    except socket.timeout:
        pass
    except Exception as e:
        print(f"서버 에러 발생: {e}")
    finally:
        try:
            conn.close()
        except Exception:
            pass