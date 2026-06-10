import serial
import time

def calculate_checksum(packet_str):
    """체크섬 계산 함수"""
    checksum_val = sum(ord(char) for char in packet_str)
    return f"{checksum_val & 0xFF:02X}"

def set_temperature(target_temp, address="01"):
    """목표 온도 설정 패킷 생성 (DWR 명령어)"""
    # 40.0도를 x10 하여 400으로 만든 뒤 16진수로 변환 (400 -> 0190)
    temp_hex = f"{int(target_temp * 10):04X}"
    
    # 0301번지(SV1)에 값 1개를 쓴다(DWR)
    command_body = f"{address}DWR,01,0301,{temp_hex}"
    checksum = calculate_checksum(command_body)
    
    # STX(0x02)와 CR LF(0x0D 0x0A) 조립
    return f"\x02{command_body}{checksum}\r\n"

if __name__ == "__main__":
    PORT_NAME = 'COM12'
    
    try:
        ser = serial.Serial(
            port=PORT_NAME,         
            baudrate=9600,       
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,  
            stopbits=serial.STOPBITS_ONE,
            timeout=2  # 대답 기다리는 시간 넉넉하게 2초 유지
        )
        
        if ser.is_open:
            print(f"[{PORT_NAME}] PASCAL 핫플레이트 연결 완료! 40.0도 설정 시작!")
            
            # 1. 찌꺼기 비우기
            ser.reset_input_buffer()
            
            # 2. 40도 설정 패킷 전송
            packet_to_send = set_temperature(35)
            print(f"▶ 송신: {repr(packet_to_send)}")
            ser.write(packet_to_send.encode('ascii'))
            
            # 3. 핫플레이트 대답 수신
            response = ser.readline()
            clean_response = response.decode('ascii', errors='ignore').replace('\x00', '').strip()
            
            # 4. 결과 출력
            if "OK" in clean_response:
                print(f"✅ 통신 완벽! 기기 온도가 40.0℃로 변경되었습니다! (응답: {clean_response})")
            elif clean_response == "":
                print("❌ 수신 타임아웃")
            else:
                print(f"⚠️ 기타 응답 발생: {clean_response}")
                
        ser.close()
        
    except Exception as e:
        print(f"통신 에러 발생: {e}")