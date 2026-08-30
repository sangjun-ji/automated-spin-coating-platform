import serial
import time

PORT = 'COM12'  # 핫플레이트 COM 포트 번호

def calculate_checksum(packet_str):
    checksum_val = sum(ord(char) for char in packet_str)
    return f"{checksum_val & 0xFF:02X}"

def test_read_format(bytesize_val, parity_val, format_name):
    print(f"🔍 [{format_name} 설정 테스트]")
    try:
        ser = serial.Serial(
            port=PORT, 
            baudrate=9600, 
            bytesize=bytesize_val, 
            parity=parity_val, 
            stopbits=serial.STOPBITS_ONE, 
            timeout=1
        )
        
        command_body = "01DRS,01,0001"
        checksum = calculate_checksum(command_body)
        packet = f"\x02{command_body}{checksum}\r\n"
        
        ser.reset_input_buffer()
        ser.write(packet.encode('ascii'))
        time.sleep(0.3)
        
        raw_bytes = ser.read_all()
        ser.close()
        
        if raw_bytes:
            print(f"   ├ RAW Hex 수신 : {raw_bytes.hex(' ')}")
            print(f"   └ ASCII 디코딩  : {repr(raw_bytes.decode('ascii', errors='ignore'))}\n")
        else:
            print("   └ ❌ 응답 없음\n")
            
    except Exception as e:
        print(f"   └ ❌ 시리얼 오류: {e}\n")

print("==================================================")
print("🧪 템코 T50 시리얼 통신 포맷(비트/패리티) 검증")
print("==================================================\n")

# 1. 8비트 Even (8E1)
test_read_format(serial.EIGHTBITS, serial.PARITY_EVEN, "8E1 (8비트 / Even)")

# 2. 7비트 Even (7E1) - 템코 ASCII 표준 규격
test_read_format(serial.SEVENBITS, serial.PARITY_EVEN, "7E1 (7비트 / Even)")

# 3. 8비트 None (8N1)
test_read_format(serial.EIGHTBITS, serial.PARITY_NONE, "8N1 (8비트 / None)")