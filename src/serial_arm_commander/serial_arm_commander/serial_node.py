# serial_arm_commander/serial_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import serial
import time

class SerialArmCommander(Node):
    def __init__(self):
        super().__init__('serial_arm_commander')

        # Parameters (set via launch)
        self.declare_parameter('port', '/dev/ttyARM')
        self.declare_parameter('baud_rate', 115200)
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud_rate').value)

        self.get_logger().info(f'node started — target serial: {self.port} @ {self.baud}')

        # Serial object (may be None if not open)
        self.ser = None
        self._open_serial()

        # Subscriber: /arm/mode expects UInt8 (1 = catch, 2 = release)
        self.sub = self.create_subscription(UInt8, '/arm/mode', self.mode_cb, 10)

        # Periodic timer to try reconnect if serial lost
        self.create_timer(2.0, self._ensure_serial_open)

    def _open_serial(self):
        if self.ser and getattr(self.ser, 'is_open', False):
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            time.sleep(0.1)
            self.get_logger().info(f'Opened serial {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().warn(f'Could not open serial {self.port}: {e}')
            self.ser = None

    def _ensure_serial_open(self):
        if not (self.ser and getattr(self.ser, 'is_open', False)):
            self._open_serial()

    def mode_cb(self, msg: UInt8):
        try:
            mode = int(msg.data)
        except Exception:
            self.get_logger().warn('Received non-integer mode')
            return

        if mode not in (1, 2):
            self.get_logger().warn(f'Ignoring invalid mode: {mode}')
            return

        payload = f'{mode}\n'.encode('ascii')
        if self.ser and getattr(self.ser, 'is_open', False):
            try:
                self.ser.write(payload)
                self.ser.flush()
                self.get_logger().info(f'Sent mode {mode} -> {self.port}')
            except Exception as e:
                self.get_logger().error(f'Write failed: {e} — closing serial and will retry')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        else:
            self.get_logger().warn(f'Serial not open; cannot send mode {mode}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialArmCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
