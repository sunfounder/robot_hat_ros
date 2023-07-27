import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32, UInt16MultiArray
from robot_hat_ros.srv import RegisterADC, RemoveADC


class TestSubscriber(Node):

    def __init__(self):
        super().__init__('test')
        self.register_adc = self.create_client(RegisterADC, 'register_adc')
        while not self.register_adc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('register_adc service not available, waiting again...')
        self.register_adc_request = RegisterADC.Request()
        self.ultrasonic_sub = self.create_subscription(
            Float32,
            'robot_hat/ultrasonic',
            self.ultrasonic_callback,
            10)
        self.grayscale_3ch_sub = self.create_subscription(
            UInt16MultiArray,
            'robot_hat/grayscale_3ch',
            self.grayscale_3ch_callback,
            10)
        # prevent unused variable warning
        self.ultrasonic_sub
        self.grayscale_3ch_sub

    def register_adc(self, channel, delay, topic, force=False):
        self.register_adc_request.channel = channel
        self.register_adc_request.delay = delay
        self.register_adc_request.topic = topic
        self.register_adc_request.force = False
        future = self.register_adc.call_async(self.register_adc_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def ultrasonic_callback(self, msg):
        self.get_logger().info('Ultrasonic read: "%f"' % msg.data)

    def grayscale_3ch_callback(self, msg):
        self.get_logger().info('Grayscale 3ch read: "%d"' % msg.data)
    
def main(args=None):
    rclpy.init(args=args)

    test = TestSubscriber()

    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()