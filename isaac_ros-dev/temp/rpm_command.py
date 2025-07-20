from pyvesc import VESC
import time

serial_port = "/host_dev/vesc"  # 실제 포트로 설정
MAX_RETRIES = 3

def initialize_vesc(port, max_retries=3, time_out=0.5):
    for attempt in range(1, max_retries + 1):
        try:
            print(f"⏳ Trying to connect to VESC (Attempt {attempt})...")
            motor = VESC(serial_port=port)
            print("✅ Connected successfully!")
            return motor
        except (ValueError, TypeError) as e:
            print(f"⚠️ Failed to initialize VESC: {e}")
            time.sleep(1)  # wait before retry
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
            break
    raise RuntimeError("VESC connection failed after multiple attempts.")

# 사용 예시
try:
    motor = initialize_vesc(serial_port)
    with motor:
        motor.set_rpm(-500)
        time.sleep(10)
        motor.set_rpm(0)
except RuntimeError as e:
    print(f"🚨 {e}")
