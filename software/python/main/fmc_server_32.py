import ctypes
import socket
import json

DLL_PATH = r"C:\Users\aned\OneDrive\Desktop\자율주행\코드\FMC4030-Dll.dll"

try:
    fmc = ctypes.WinDLL(DLL_PATH)
except Exception as e:
    print(f"❌ DLL 로드 실패: {e}")
    exit()

# ==========================================
# C언어 함수 규격 설정 (기존 성공 설정 유지)
# ==========================================
fmc_open = getattr(fmc, "_FMC4030_Open_Device@12")
fmc_jog = getattr(fmc, "_FMC4030_Jog_Single_Axis@28")
fmc_get_pos = getattr(fmc, "_FMC4030_Get_Axis_Current_Pos@12")
fmc_check_stop = getattr(fmc, "_FMC4030_Check_Axis_Is_Stop@8")
fmc_home = getattr(fmc, "_FMC4030_Home_Single_Axis@24")

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

# ==========================================
# 장비 연결
# ==========================================
device_id = 0
ip = b"192.168.0.30"
port = 8088

print("장비 연결을 시도합니다...")
res = fmc_open(device_id, ip, port)
if res < 0:
    print(f"❌ 장비 연결 실패 (에러 코드: {res})")
    exit()
print("✅ 기계 연결 성공!")

# ==========================================
# 소켓 서버 구동 (포트 재사용 옵션 추가)
# ==========================================
HOST = '127.0.0.1'
PORT = 50000
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # 서버 재시작 시 포트 꼬임 방지
server.bind((HOST, PORT))
server.listen(5)
print(f"🚀 32비트 서버 가동 완료. 명령 대기 중... (포트: {PORT})")

while True:
    conn, addr = server.accept()
    try:
        data = conn.recv(1024).decode('utf-8')
        if not data:
            conn.close()
            continue
            
        cmd_data = json.loads(data)
        action = cmd_data.get('action')
        
        # 1. 절대/상대 이동 명령 (mode: 2 -> 절대좌표, mode: 1 -> 상대좌표)
        if action == 'move':
            axis = cmd_data['axis']
            pos = cmd_data['pos']
            speed = cmd_data['speed']
            mode = cmd_data.get('mode', 2) # 기본값을 절대좌표(2)로 설정
            
            res_code = fmc_jog(
                device_id, axis, 
                ctypes.c_float(pos), ctypes.c_float(speed), 
                ctypes.c_float(300.0), ctypes.c_float(300.0), mode
            )
            
            current_pos = ctypes.c_float(0.0)
            fmc_get_pos(device_id, axis, ctypes.byref(current_pos))
            
            response = {"status": "OK", "res_code": res_code, "current_pos": current_pos.value}
            conn.sendall(json.dumps(response).encode('utf-8'))
            
        # 2. 정지 확인 명령
        elif action == 'check_stop':
            axis = cmd_data['axis']
            is_stop = fmc_check_stop(device_id, axis)
            
            current_pos = ctypes.c_float(0.0)
            fmc_get_pos(device_id, axis, ctypes.byref(current_pos))
            
            response = {"is_stop": is_stop, "current_pos": current_pos.value}
            conn.sendall(json.dumps(response).encode('utf-8'))
            
        # 3. 원점 호밍 명령
        elif action == 'home':
            axis = cmd_data['axis']
            speed = cmd_data.get('speed', 50.0)
            acc_dec = cmd_data.get('acc_dec', 100.0)
            fall_step = cmd_data.get('fall_step', 5.0) 
            direction = cmd_data.get('direction', 2) 
            
            res_code = fmc_home(
                device_id, axis, 
                ctypes.c_float(speed), ctypes.c_float(acc_dec), 
                ctypes.c_float(fall_step), direction
            )
            response = {"status": "OK", "res_code": res_code}
            conn.sendall(json.dumps(response).encode('utf-8'))
            
    except Exception as e:
        print(f"⚠️ 서버 에러 발생: {e}")
    finally:
        conn.close()