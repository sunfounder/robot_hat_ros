import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import UInt16MultiArray, Int32
from robot_hat import Ultrasonic, Pin
from robot_hat_ros.srv import RegisterADC


class Grayscale3chPublisher(Node):

    def __init__(self):
        super().__init__('grayscale_3ch_publisher')
        self.left = "A0"
        self.middle = "A1"
        self.right = "A2"
        self.values = []
        left_descriptor = ParameterDescriptor(description='Set the left channel pin')
        middle_descriptor = ParameterDescriptor(description='Set the middle channel pin')
        right_descriptor = ParameterDescriptor(description='Set the right channel pin')
        self.declare_parameter('left', self.left, left_descriptor)
        self.declare_parameter('middle', self.middle, middle_descriptor)
        self.declare_parameter('right', self.right, right_descriptor)
        self.publisher_ = self.create_publisher(UInt16MultiArray, 'robot_hat/grayscale_3ch', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.register_adc = self.create_client(RegisterADC, 'register_adc')
        while not self.register_adc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('register_adc service not available, waiting again...')
        self.register_adc_request = RegisterADC.Request()
        self.register_adc(self.left, self.timer_period, 'robot_hat/grayscale/left')
        self.register_adc(self.middle, self.timer_period, 'robot_hat/grayscale/middle')
        self.register_adc(self.right, self.timer_period, 'robot_hat/grayscale/right')
        self.left_sub = self.create_subscription(Int32, 'robot_hat/grayscale/left', self.left_callback, 10)
        self.middle_sub = self.create_subscription(Int32, 'robot_hat/grayscale/middle', self.middle_callback, 10)
        self.right_sub = self.create_subscription(Int32, 'robot_hat/grayscale/right', self.right_callback, 10)

    def left_callback(self, msg):
        self.get_logger().info('Left read: "%d"' % msg.data)
        self.values[0] = msg.data

    def middle_callback(self, msg):
        self.get_logger().info('Middle read: "%d"' % msg.data)
        self.values[1] = msg.data

    def right_callback(self, msg):
        self.get_logger().info('Right read: "%d"' % msg.data)
        self.values[2] = msg.data

    def check_pin_changed(self):
        left = self.get_parameter('left').get_parameter_value().string_value
        middle = self.get_parameter('middle').get_parameter_value().string_value
        right = self.get_parameter('right').get_parameter_value().string_value
        if left != self.left:
            self.register_adc(self.left, self.timer_period, 'robot_hat/grayscale/left')
            self.left = left
            self.get_logger().info('Left channel pin changed to: "%s"' % self.left)
        if middle != self.middle:
            self.register_adc(self.middle, self.timer_period, 'robot_hat/grayscale/middle')
            self.middle = middle
            self.get_logger().info('Middle channel pin changed to: "%s"' % self.middle)
        if right != self.right:
            self.register_adc(self.right, self.timer_period, 'robot_hat/grayscale/right')
            self.right = right
            self.get_logger().info('Right channel pin changed to: "%s"' % self.right)

    def register_adc(self, channel, delay, topic):
        self.register_adc_request.channel = channel
        self.register_adc_request.delay = delay
        self.register_adc_request.topic = topic
        self.register_adc_request.force = False
        future = self.register_adc.call_async(self.register_adc_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def timer_callback(self):
        self.check_pin_changed()
        msg = UInt16MultiArray()
        msg.data = self.values
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    grayscale_3ch_publisher = Grayscale3chPublisher()

    rclpy.spin(grayscale_3ch_publisher)

    grayscale_3ch_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()