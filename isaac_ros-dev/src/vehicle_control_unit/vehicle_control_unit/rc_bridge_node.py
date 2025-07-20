import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import threading
import time

class RCBridgeNode(Node):
    def __init__(self):
        super().__init__('rc_bridge_node')
        
        # Thread-safe servo command storage
        self.servo_lock = threading.Lock()
        self.latest_servo_cmd = 90  # default center angle
        
        # Serial connection
        self.serial_lock = threading.Lock()
        self.arduino_serial = None
        self.serial_port = '/host_dev/arduino'
        self.serial_baud = 57600
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        
        # Initialize serial connection
        self.connect_arduino()
        
        # Subscribe to servo angle command
        self.create_subscription(Int32, 'servo_cmd', self.update_servo_command, 10)
        
        # Publishers for RC channels from Arduino
        self.publisher_ch1 = self.create_publisher(Int32, 'rc/ch1', 10)
        self.publisher_ch2 = self.create_publisher(Int32, 'rc/ch2', 10)
        self.publisher_ch3 = self.create_publisher(Int32, 'rc/ch3', 10)
        
        # Periodic sender (50 Hz)
        self.create_timer(0.02, self.send_servo_periodic)
        
        # Start background thread to read from Arduino
        self.read_thread = threading.Thread(target=self.read_serial_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        # Periodic connection check (every 5 seconds)
        self.create_timer(5.0, self.check_connection)

    def connect_arduino(self):
        """Initialize or reconnect to Arduino"""
        with self.serial_lock:
            if self.arduino_serial:
                try:
                    self.arduino_serial.close()
                except:
                    pass
                self.arduino_serial = None
            
            try:
                self.arduino_serial = serial.Serial(
                    self.serial_port, 
                    self.serial_baud, 
                    timeout=1,
                    write_timeout=1
                )
                self.get_logger().info(f"✅ Arduino connected at {self.serial_port}")
                self.reconnect_attempts = 0
                return True
            except Exception as e:
                self.reconnect_attempts += 1
                if self.reconnect_attempts <= self.max_reconnect_attempts:
                    self.get_logger().warn(f"⚠️ Arduino connection failed (attempt {self.reconnect_attempts}): {e}")
                else:
                    self.get_logger().error(f"❌ Arduino connection failed after {self.max_reconnect_attempts} attempts: {e}")
                return False

    def is_connected(self):
        """Check if Arduino is connected"""
        with self.serial_lock:
            return self.arduino_serial is not None and self.arduino_serial.is_open

    def check_connection(self):
        """Periodic connection health check"""
        if not self.is_connected() and self.reconnect_attempts < self.max_reconnect_attempts:
            self.get_logger().info("🔄 Attempting to reconnect to Arduino...")
            self.connect_arduino()

    def update_servo_command(self, msg: Int32):
        """Update servo angle command (thread-safe)"""
        angle = max(0, min(180, msg.data))
        with self.servo_lock:
            self.latest_servo_cmd = angle
        self.get_logger().debug(f"Servo command updated: {angle}°")

    def send_servo_periodic(self):
        """Send servo command to Arduino at 50Hz"""
        if not self.is_connected():
            return
            
        with self.servo_lock:
            current_angle = self.latest_servo_cmd
            
        command = f'ANGLE:{current_angle}\n'
        
        try:
            with self.serial_lock:
                if self.arduino_serial and self.arduino_serial.is_open:
                    self.arduino_serial.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f"⚠️ Serial write error: {e}")
            # Mark connection as failed to trigger reconnection
            with self.serial_lock:
                if self.arduino_serial:
                    try:
                        self.arduino_serial.close()
                    except:
                        pass
                    self.arduino_serial = None
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to send servo command: {e}")

    def parse_rc_channels(self, line):
        """Parse RC channel data from Arduino"""
        try:
            if not line.startswith("CH1:"):
                return None, None, None
                
            # Expected format: "CH1:1500 CH2:1000 CH3:2000"
            parts = line.split()
            channels = {}
            
            for part in parts:
                if ':' in part:
                    key, value_str = part.split(':', 1)
                    if value_str.isdigit():
                        channels[key] = int(value_str)
            
            ch1 = channels.get('CH1')
            ch2 = channels.get('CH2') 
            ch3 = channels.get('CH3')
            
            # Validate channel values (typical RC range: 1000-2000)
            def validate_channel(value):
                return value is not None and 800 <= value <= 2200
            
            if all(validate_channel(ch) for ch in [ch1, ch2, ch3]):
                return ch1, ch2, ch3
            else:
                self.get_logger().debug(f"Invalid channel values: CH1={ch1}, CH2={ch2}, CH3={ch3}")
                return None, None, None
                
        except Exception as e:
            self.get_logger().debug(f"Parse error for line '{line}': {e}")
            return None, None, None

    def read_serial_loop(self):
        """Background thread to continuously read from Arduino"""
        while rclpy.ok():
            if not self.is_connected():
                time.sleep(0.1)
                continue
                
            try:
                with self.serial_lock:
                    if self.arduino_serial and self.arduino_serial.is_open:
                        if self.arduino_serial.in_waiting > 0:
                            line = self.arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                        else:
                            line = ""
                    else:
                        line = ""
                
                if line:
                    ch1, ch2, ch3 = self.parse_rc_channels(line)
                    
                    if ch1 is not None and ch2 is not None and ch3 is not None:
                        # Publish RC channel values
                        self.publisher_ch1.publish(Int32(data=ch1))
                        self.publisher_ch2.publish(Int32(data=ch2))
                        self.publisher_ch3.publish(Int32(data=ch3))
                        self.get_logger().debug(f"Published RC: CH1={ch1}, CH2={ch2}, CH3={ch3}")
                    elif line.startswith("CH1:"):
                        self.get_logger().debug(f"Received malformed RC data: {line}")
                else:
                    # No data available, small delay to prevent busy waiting
                    time.sleep(0.001)
                    
            except serial.SerialException as e:
                self.get_logger().warn(f"⚠️ Serial read error: {e}")
                # Mark connection as failed
                with self.serial_lock:
                    if self.arduino_serial:
                        try:
                            self.arduino_serial.close()
                        except:
                            pass
                        self.arduino_serial = None
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().warn(f"⚠️ Arduino read error: {e}")
                time.sleep(0.1)

    def cleanup(self):
        """Clean up resources"""
        self.get_logger().info("🧹 Cleaning up RCBridgeNode...")
        with self.serial_lock:
            if self.arduino_serial:
                try:
                    self.arduino_serial.close()
                    self.get_logger().info("📡 Arduino serial connection closed")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Error closing serial: {e}")
                self.arduino_serial = None

def main(args=None):
    rclpy.init(args=args)
    node = RCBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down RCBridgeNode...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()