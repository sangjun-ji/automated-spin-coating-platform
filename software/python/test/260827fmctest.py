# =====================================================================
# fmc_latency_test.py   -  FMC4030 폴링 지연 진단 (독립 실행)
#
#   python fmc_latency_test.py
#
# fmc_server_32.py 가 떠 있어야 한다. 260826handshake.py 는 건드리지 않는다.
#
# [TEST 1] 무동작 왕복시간 측정   <-- 기본값. 축이 전혀 움직이지 않는다.
#          check_stop 을 200회 날려서 폴링 1회 비용을 잰다.
#          이 값이 30ms 를 넘으면 FMC_POLL_INTERVAL=0.03 은 무의미하고,
#          컨트롤러를 처리 능력 이상으로 두드리고 있다는 뜻이다.
#
# [TEST 2] 실제 이동 프로파일링   <-- ENABLE_MOVE_TEST = True 로 켜야 실행됨.
#          지정 축을 상대좌표로 왕복시키며 시간을 분해한다.
#          !! 축이 실제로 움직인다. 아래 주의사항을 반드시 읽을 것 !!
# =====================================================================

import socket
import json
import time
import statistics

# ---------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------
FMC_HOST = '127.0.0.1'
FMC_PORT = 50000
SOCK_TIMEOUT = 3.0

# --- TEST 1 (무동작) ---
PROBE_AXIS   = 0        # 상태를 읽을 축 (0=X, 1=Y, 2=Z). 읽기만 하므로 안전.
PROBE_COUNT  = 200      # 측정 횟수
PROBE_GAP    = 0.03     # 측정 간격(초). 현재 코드의 FMC_POLL_INTERVAL 과 동일하게.

# --- TEST 2 (실제 이동) ---
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !! 아래를 True 로 바꾸면 축이 실제로 움직인다.                      !!
# !! 실행 전 반드시 확인할 것:                                        !!
# !!   - 그리퍼에 시료가 물려 있지 않은가                             !!
# !!   - 피펫 Z축이 위로 대피해 있는가                                !!
# !!   - MOVE_AXIS 의 이동 경로 +-MOVE_DIST 안에 간섭물이 없는가       !!
# !!   - 리밋 스위치까지 여유가 있는가                                !!
# !! 상대좌표(mode=1) 로 +MOVE_DIST 갔다가 -MOVE_DIST 로 되돌아온다.   !!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
ENABLE_MOVE_TEST = False

MOVE_AXIS    = 0        # 0=X, 1=Y, 2=Z.  처음에는 X(0) 로만 시험할 것.
MOVE_DIST    = 30.0     # 편도 이동 거리(mm). 작게 시작.
MOVE_SPEED   = 300.0    # mm/s
MOVE_CYCLES  = 3        # 왕복 횟수

# 폴링 파라미터 (260826handshake.py 와 동일한 값으로 두고 비교)
POLL_INTERVAL = 0.03
MOTION_GRACE  = 0.20
STOP_CONFIRM  = 2

AXIS_NAME = {0: "X", 1: "Y", 2: "Z"}


# ---------------------------------------------------------------------
# 통신
# ---------------------------------------------------------------------
def send_fmc(command_dict):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(SOCK_TIMEOUT)
        s.connect((FMC_HOST, FMC_PORT))
        s.sendall(json.dumps(command_dict).encode('utf-8'))
        return json.loads(s.recv(1024).decode('utf-8'))


# ---------------------------------------------------------------------
# TEST 1 : 무동작 왕복시간
# ---------------------------------------------------------------------
def test_roundtrip():
    print("=" * 68)
    print(f"[TEST 1] 무동작 왕복시간 측정  (axis {PROBE_AXIS}, {PROBE_COUNT}회, "
          f"간격 {PROBE_GAP*1000:.0f}ms)")
    print("         축은 움직이지 않는다. 상태만 읽는다.")
    print("=" * 68)

    rtts = []
    values = {}
    errors = 0

    for i in range(PROBE_COUNT):
        t = time.time()
        try:
            resp = send_fmc({"action": "check_stop", "axis": PROBE_AXIS})
            rtts.append((time.time() - t) * 1000.0)
            v = resp.get("is_stop")
            values[v] = values.get(v, 0) + 1
        except Exception as e:
            errors += 1
            if errors <= 3:
                print(f"   [{i}] 통신 에러: {e}")
        time.sleep(PROBE_GAP)

    if not rtts:
        print("\n측정 실패. fmc_server_32.py 가 실행 중인지 확인할 것.")
        return None

    rtts.sort()
    avg = statistics.mean(rtts)
    p50 = rtts[len(rtts) // 2]
    p95 = rtts[int(len(rtts) * 0.95)]

    print(f"\n  왕복시간   평균 {avg:6.2f} ms")
    print(f"             중앙 {p50:6.2f} ms")
    print(f"             p95  {p95:6.2f} ms")
    print(f"             최대 {rtts[-1]:6.2f} ms   최소 {rtts[0]:6.2f} ms")
    print(f"  is_stop 반환값 분포: {values}")
    print(f"  통신 에러: {errors}회")

    print("\n  --- 판정 ---")
    if avg < 10:
        print(f"  통신은 빠르다({avg:.1f}ms). 폴링 경로는 병목이 아니다.")
        print("  -> 원인은 다른 곳. TEST 2 로 이동 시간 자체를 확인할 것.")
    elif avg < 30:
        print(f"  통신이 다소 느리다({avg:.1f}ms). 폴링 1회에 {avg:.0f}ms 가 든다는 뜻.")
        print(f"  -> STOP_CONFIRM=2 이면 정지 후 최소 {avg*2:.0f}ms 를 더 기다린다.")
        print("  -> STOP_CONFIRM 을 1 로 낮춰볼 것.")
    else:
        print(f"  통신이 느리다({avg:.1f}ms > 폴링주기 {PROBE_GAP*1000:.0f}ms).")
        print("  -> POLL_INTERVAL 이 무의미하다. 루프가 통신 속도로만 돈다.")
        print("  -> 컨트롤러를 처리 능력 이상으로 두드리는 중일 가능성이 높다.")
        print("  -> 조치 (파일 하단 [개선안] 참고):")
        print("       (1) 서버에 위치조회를 생략한 check_stop_fast 액션 추가")
        print("       (2) POLL_INTERVAL 을 0.05~0.08 로")
        print("       (3) STOP_CONFIRM 을 1 로")

    if set(values.keys()) - {0, 1}:
        print(f"\n  [주의] is_stop 이 0/1 이 아닌 값으로 온다: "
              f"{set(values.keys()) - {0, 1}}")
        print("         SDK 문서상 이 함수는 0/1 만 반환한다. 통신 이상 신호.")

    return avg


# ---------------------------------------------------------------------
# TEST 2 : 실제 이동 프로파일링
# ---------------------------------------------------------------------
def wait_and_profile(axis, timeout=60.0):
    """260826handshake.py 의 wait_for_fmc_stop 과 동일한 판정 + 시간 분해"""
    t0 = time.time()
    motion_seen = False
    stop_count = 0
    last_pos = 0.0
    polls = 0
    rtt_sum = 0.0
    t_first_motion = None
    t_last_motion = None
    odd = {}

    while True:
        try:
            t = time.time()
            resp = send_fmc({"action": "check_stop", "axis": axis})
            rtt_sum += time.time() - t
            polls += 1

            is_stop = resp.get("is_stop")
            last_pos = resp.get("current_pos", 0.0)
            elapsed = time.time() - t0

            if is_stop == 1:
                stop_count += 1
                if motion_seen and stop_count >= STOP_CONFIRM:
                    break
                if (not motion_seen) and elapsed >= MOTION_GRACE:
                    break
            elif is_stop == 0:
                if not motion_seen:
                    t_first_motion = elapsed
                motion_seen = True
                t_last_motion = elapsed
                stop_count = 0
            else:
                odd[is_stop] = odd.get(is_stop, 0) + 1
                stop_count = 0

            if elapsed > timeout:
                print("      타임아웃!")
                break

            time.sleep(POLL_INTERVAL)

        except Exception as e:
            print(f"      통신 에러: {e}")
            break

    total = time.time() - t0
    startup = t_first_motion if t_first_motion is not None else 0.0
    move = ((t_last_motion - t_first_motion)
            if (t_first_motion is not None and t_last_motion is not None) else 0.0)
    confirm = total - (t_last_motion if t_last_motion is not None else startup)

    return {
        "total": total, "startup": startup, "move": move, "confirm": confirm,
        "polls": polls, "rtt": (rtt_sum / polls * 1000.0) if polls else 0.0,
        "pos": last_pos, "moved": motion_seen, "odd": odd,
    }


def test_move():
    name = AXIS_NAME.get(MOVE_AXIS, str(MOVE_AXIS))

    print("\n" + "=" * 68)
    print(f"[TEST 2] 실제 이동 프로파일링")
    print(f"         축 {name}({MOVE_AXIS}), 상대이동 +-{MOVE_DIST}mm, "
          f"속도 {MOVE_SPEED}mm/s, {MOVE_CYCLES}회 왕복")
    print("=" * 68)

    try:
        cur = send_fmc({"action": "check_stop", "axis": MOVE_AXIS})
    except Exception as e:
        print(f"서버 연결 실패: {e}")
        return
    print(f"\n  현재 {name}축 위치: {cur.get('current_pos', 0.0):.2f} mm")
    print(f"  이동 범위: {cur.get('current_pos', 0.0):.2f} "
          f"~ {cur.get('current_pos', 0.0) + MOVE_DIST:.2f} mm")

    print(f"\n  이 범위에 간섭물이 없고, 그리퍼가 비어 있고,")
    print(f"  피펫 Z축이 대피해 있는지 확인했는가?")
    ans = input("  진행하려면 GO 를 입력 (그 외에는 취소): ").strip()
    if ans != "GO":
        print("  취소함.")
        return

    results = []
    for c in range(MOVE_CYCLES):
        for sign, label in ((+1, "정방향"), (-1, "역방향")):
            d = MOVE_DIST * sign
            print(f"\n  [{c+1}/{MOVE_CYCLES}] {label} {d:+.1f}mm")
            r = send_fmc({"action": "move", "axis": MOVE_AXIS,
                          "pos": d, "speed": MOVE_SPEED, "mode": 1})
            if r.get("res_code") != 0:
                print(f"      이동 명령 거부됨 (res_code={r.get('res_code')})")
                print("      SDK 문서: 같은 축이 아직 이동 중이면 명령이 무시된다.")
                continue

            p = wait_and_profile(MOVE_AXIS)
            results.append(p)
            print(f"      total={p['total']*1000:6.0f}ms  "
                  f"startup={p['startup']*1000:5.0f}  "
                  f"move={p['move']*1000:6.0f}  "
                  f"confirm={p['confirm']*1000:5.0f}  |  "
                  f"polls={p['polls']:3d}  rtt={p['rtt']:5.1f}ms  "
                  f"pos={p['pos']:.2f}mm"
                  + (f"  ODD={p['odd']}" if p['odd'] else ""))
            time.sleep(0.3)

    if not results:
        return

    print("\n" + "-" * 68)
    print("  평균")
    for k, label in (("total", "전체"), ("startup", "출발까지"),
                     ("move", "이동중"), ("confirm", "정지확인")):
        v = statistics.mean(r[k] for r in results) * 1000.0
        print(f"    {label:8s} {v:7.1f} ms")
    print(f"    {'폴링RTT':8s} {statistics.mean(r['rtt'] for r in results):7.1f} ms")

    avg_total = statistics.mean(r["total"] for r in results)
    avg_move = statistics.mean(r["move"] for r in results)
    overhead = avg_total - avg_move

    print("\n  --- 판정 ---")
    print(f"  소프트웨어 오버헤드 = {overhead*1000:.0f}ms "
          f"(전체의 {overhead/avg_total*100:.0f}%)")
    if overhead < 0.15:
        print("  오버헤드가 작다. 체감 지연은 기구 이동 시간 자체다.")
        print(f"  -> {MOVE_DIST}mm 를 {avg_move*1000:.0f}ms 에 이동 중.")
        print("     서버의 가감속(현재 300mm/s^2 하드코딩)이 병목일 가능성이 높다.")
        print("     [개선안] C-2 참고.")
    else:
        print("  오버헤드가 크다. 폴링 경로를 손보면 줄일 수 있다.")
        print("  [개선안] C-1, C-3 참고.")


# ---------------------------------------------------------------------
if __name__ == "__main__":
    print("FMC4030 폴링 지연 진단\n")
    test_roundtrip()

    if ENABLE_MOVE_TEST:
        test_move()
    else:
        print("\n" + "=" * 68)
        print("[TEST 2] 건너뜀 (ENABLE_MOVE_TEST = False)")
        print("         실제 이동 프로파일링을 하려면 파일 상단에서 True 로 바꿀 것.")
        print("         축이 실제로 움직이므로 주의사항을 먼저 읽을 것.")
        print("=" * 68)


# =====================================================================
# [개선안]  진단 결과에 따라 적용
# =====================================================================
#
# C-1. 폴링당 컨트롤러 왕복을 2회 -> 1회로
#
#      현재 서버의 check_stop 은 fmc_check_stop 과 fmc_get_pos 를 둘 다
#      호출한다. 통신이 전부 이더넷 왕복이므로 폴링 1회에 왕복 2회를 쓴다.
#      위치는 완료 시점에 한 번만 있으면 된다.
#
#      fmc_server_32.py 의 while 루프에 추가:
#
#          elif action == 'check_stop_fast':
#              is_stop = fmc_check_stop(device_id, cmd_data['axis'])
#              conn.sendall(json.dumps({"is_stop": is_stop}).encode('utf-8'))
#
#      폴링은 check_stop_fast 로, 루프 종료 후 위치는 check_stop 으로 1회.
#
#
# C-2. 가감속을 명령 파라미터로 (실제 시간 단축의 핵심)
#
#      SDK: Jog_Single_Axis(id, axis, pos, speed, acc, dec, mode)
#           acc/dec 단위 mm/s^2
#
#      fmc_server_32.py 87~91행이 acc=dec=300.0 으로 고정되어 있다.
#      가속도 300mm/s^2 로 300mm/s 에 도달하려면 가속 구간만 150mm 가 필요하다.
#      레시피의 이동 거리는 대부분 100~200mm 이므로 speed=300 은 실제로
#      도달하지 못하고, speed 를 150 에서 300 으로 올린 효과가 거의 없다.
#
#          acc = cmd_data.get('acc', 300.0)
#          dec = cmd_data.get('dec', 300.0)
#          res_code = fmc_jog(device_id, axis,
#                             ctypes.c_float(pos), ctypes.c_float(speed),
#                             ctypes.c_float(acc), ctypes.c_float(dec), mode)
#
#      이 스크립트의 MOVE_SPEED 와 함께 acc 를 500 -> 800 -> 1200 으로
#      올려가며 TEST 2 의 move 시간이 실제로 줄어드는지 확인할 것.
#      Z축은 중력 방향이므로 보수적으로. 시료 흔들림도 함께 볼 것.
#
#
# C-3. X, Y 동시 발행
#
#      SDK: "서로 다른 축은 여러 번 시작할 수 있다.
#            같은 축의 이전 동작이 끝나지 않았으면 그 명령은 무시된다."
#      -> 다른 축을 동시에 띄우는 것은 문서상 허용된다.
#
#          send_fmc({"action":"move","axis":0,"pos":x,"speed":300,"mode":2})
#          send_fmc({"action":"move","axis":1,"pos":y,"speed":300,"mode":2})
#          wait_for_fmc_stop(0)
#          wait_for_fmc_stop(1)
#
#      단 경로가 대각선이 되므로 간섭 확인 필요. 현재 레시피는 X 스텝과
#      Y 스텝이 따로 있으므로 합칠 수 있는지 먼저 검토할 것.
#
#      더 나아가려면 Line_2Axis(보간 이동) 를 서버에 추가하면 X,Y 가
#      협조 제어로 하나의 직선 경로를 그린다.
#
#
# C-4. 완료 판정을 속도 기반으로
#
#      SDK 에 Get_Axis_Current_Speed(id, axis, float* speed) 가 있다.
#      그리퍼에서 reg13(모터 속도) 로 정지를 판정하도록 바꾼 것과 같은 논리로,
#      is_stop 플래그보다 speed==0 이 더 직접적인 증거다.
#
#      Get_Machine_Status 는 3축 위치와 속도를 한 번에 준다.
#      축마다 따로 물을 필요가 없어 왕복이 크게 줄지만,
#      구조체 레이아웃이 FMC4030-Dll.h 에 있으므로 그 헤더가 필요하다.
#
#
# C-5. 서버 종료 시 Close_Device
#
#      SDK: "프로그램 종료 전 반드시 호출해 리소스를 해제해야 하며,
#            그렇지 않으면 다음 연결이 실패한다."
#      현재 fmc_server_32.py 는 호출하지 않는다.
#      서버를 껐다 켰을 때 연결이 안 되는 증상이 있었다면 이것이 원인이다.
#
#          import atexit
#          fmc_close = getattr(fmc, "_FMC4030_Close_Device@4")
#          fmc_close.argtypes = [ctypes.c_long]
#          fmc_close.restype  = ctypes.c_long
#          atexit.register(lambda: fmc_close(device_id))
#
#      (@4 는 stdcall 데코레이션. 실제 이름은 dumpbin 등으로 확인 필요.)
# =====================================================================