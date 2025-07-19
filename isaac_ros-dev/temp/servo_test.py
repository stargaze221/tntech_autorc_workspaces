import serial
import time

# 아두이노가 연결된 포트 (ls /dev/ttyUSB* 또는 /dev/ttyACM* 로 확인)
SERIAL_PORT = "/dev/arduino"
BAUD_RATE = 9600

# 보낼 서보 각도 (0 ~ 180)
servo_angles = [60, 90, 120]

try:
    # 아두이노와 시리얼 통신 열기
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # 아두이노 초기화 대기

    print("✅ Connected to Arduino")

    for angle in servo_angles:
        command = f"{angle}\n"
        print(f"➡️ Sending angle: {angle}")
        ser.write(command.encode('utf-8'))
        time.sleep(2)

    print("✅ Servo control via Arduino complete.")

    ser.close()

except Exception as e:
    print("❌ Failed to communicate with Arduino:", e)
