from pyvesc import VESC
import time

try:
    # Wait before opening the port (just in case device needs to settle)
    time.sleep(1)

    with VESC(serial_port='/dev/vesc') as vesc:
        print("Connected to VESC. Starting test...")

        # Small delay to ensure firmware is ready
        time.sleep(0.5)

        # Forward rotation
        vesc.set_rpm(2000)
        print("Spinning forward...")
        time.sleep(2)

        # Stop
        vesc.set_rpm(0)
        time.sleep(1)

        # Reverse rotation
        vesc.set_rpm(-2000)
        print("Spinning in reverse...")
        time.sleep(2)

        # Final stop
        vesc.set_rpm(0)
        print("Motor stopped.")

except ValueError as e:
    print(f"⚠️ Failed to communicate with VESC (firmware issue?): {e}")

except Exception as e:
    print(f"❌ Unexpected error: {e}")
