from pyvesc import VESC

self.motor = VESC(serial_port='/dev/ttyACM0')
self.motor.set_rpm(rpm)
