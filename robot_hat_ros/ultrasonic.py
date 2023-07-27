import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Float32
from robot_hat import Ultrasonic, Pin


class UltrasonicPublisher(Node):

    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.trig = "D0"
        self.echo = "D1"
        trig_descriptor = ParameterDescriptor(description='Set the Trig pin')
        echo_descriptor = ParameterDescriptor(description='Set the Echo pin')
        self.declare_parameter('trig', self.trig, trig_descriptor)
        self.declare_parameter('echo', self.echo, echo_descriptor)
        self.publisher_ = self.create_publisher(Float32, 'robot_hat/ultrasonic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ultrasonic = Ultrasonic(Pin(self.trig), Pin(self.echo))

    def check_pin_changed(self):
        trig = self.get_parameter('trig').get_parameter_value().string_value
        echo = self.get_parameter('echo').get_parameter_value().string_value
        if trig != self.trig or echo != self.echo:
            self.trig = trig
            self.echo = echo
            self.ultrasonic = Ultrasonic(Pin(self.trig), Pin(self.echo))
            self.get_logger().info('Ultrasonic pins changed to: "%s" and "%s"' % (self.trig, self.echo))

    def timer_callback(self):
        self.check_pin_changed()
        msg = Float32()
        distance = self.ultrasonic.read()
        if distance != -1:
            msg.data = distance
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%f"' % msg.data)
        else:
            self.get_logger().info('Failed to read distance')


def main(args=None):
    rclpy.init(args=args)

    ultrasonic_publisher = UltrasonicPublisher()

    rclpy.spin(ultrasonic_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ultrasonic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()