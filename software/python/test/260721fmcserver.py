import ctypes
import socket
import json

# 1. 32비트 DLL 로드
dll_path = r"C:\Users\aned\OneDrive\Desktop\자율주행\코드\FMC4030-Dll.dll"
fmc = ctypes.WinDLL(dll_path) 

# 2. 함수 지정
fmc_open_device = getattr(fmc, "_FMC4030_Open_Device@12")
fmc_jog_single_axis = getattr(fmc, "_FMC4030_Jog_Single_Axis@28")

# 3. C언어 원형 규격에 맞게 데이터 타입 엄격 고정
fmc_open_device.argtypes = [ctypes.c_long, ctypes.c_char_p, ctypes.c_long]
fmc_open_device.restype = ctypes.c_long

fmc_jog_single_axis.argtypes = [
    ctypes.c_long, ctypes.c_long, ctypes.c_float, ctypes.c_float, 
    ctypes.c_float, ctypes.c_float, ctypes.c_long
]

# 4. 기계 연결 시도 (IP 문자열 버퍼 안전하게 생성)
device_id = ctypes.c_long(0)
ip_address_buf = ctypes.create_string_buffer(b"192.168.0.30")
port = ctypes.c_long(8088)

print("기계 연결 시도 중...")

# 안전한 버퍼 포인터로 전달
result = fmc_open_device(device_id, ctypes.cast(ip_address_buf, ctypes.c_char_p), port)
print(f"연결 리턴 코드: {result}")

if result < 0:
    print("❌ 기계 연결 실패!")
    exit()

print("✅ 기계 연결 성공! (명령 대기 중...)")

# 5. 소켓 서버 구동
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('127.0.0.1', 50000))
server.listen(1)

while True:
    conn, addr = server.accept()
    data = conn.recv(1024).decode('utf-8')
    if data:
        cmd = json.loads(data)
        axis = cmd['axis']
        pos = cmd['pos']
        speed = cmd['speed']
        
        print(f"명령 수신: {axis}번 축을 {pos}mm 위치로 {speed}mm/s 속도로 이동")
        
        fmc_jog_single_axis(
            0, axis, 
            ctypes.c_float(pos), ctypes.c_float(speed), 
            ctypes.c_float(50.0), ctypes.c_float(50.0), 0
        )
        conn.sendall(b"OK")
    conn.close()