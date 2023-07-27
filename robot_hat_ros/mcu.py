from robot_hat import ADC, PWM, Servo
from robot_hat_ros.srv import RegisterADC, RemoveADC
from std_msgs.msg import Int32

import rclpy
from rclpy.node import Node
import time

class MCUService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.register_adc = self.create_service(RegisterADC, 'register_adc', self.register_adc_callback)
        self.remove_adc = self.create_service(RemoveADC, 'remove_adc', self.remove_adc_callback)
        self.adc = {}
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        for channel in self.adc:
            delay = self.adc[channel]['delay']
            last_read = self.adc[channel]['last_read']
            if time.time() - last_read < delay:
                continue
            adc = self.adc[channel]['adc']
            publisher = self.adc[channel]['publisher']
            topic = self.adc[channel]['topic']
            msg = Int32()
            msg.data = adc.read()
            publisher.publish(msg)
            self.adc[channel]['last_read'] = time.time()
            self.get_logger().info(f'Publishing {msg.data} to {topic}')

    def register_adc_callback(self, request, response):
        channel = request.channel
        delay = request.delay
        force = request.force
        topic = request.topic
        if channel in self.adc:
            if force:
                self.destroy_publisher(self.adc[channel]['publisher'])
            else:
                self.get_logger().error('Channel %d already registered' % channel)
                response.status = False
                return response

        elif channel not in ['A0', 'A1', 'A2', 'A3', 'A4']:
            self.get_logger().error('Invalid channel %d' % channel)
            response.status = False
            return response
        self.adc[channel] = {
            'adc': ADC(channel),
            'delay': delay,
            'topic': topic,
            'publisher': self.create_publisher(Int32, topic, 10),
            'last_read': time.time(),
        }
        self.get_logger().info(f'Register ADC\n  channel: {channel} delay: {delay}, topic: {topic}')
        response.status = True
        return response

    def remove_adc_callback(self, request, response):
        channel = request.channel
        if channel in self.adc:
            self.destroy_publisher(self.adc[channel]['publisher'])
            del self.adc[channel]
            self.get_logger().info(f'Removed ADC channel {channel}')
            response.status = True
            return response
        else:
            self.get_logger().error(f'Channel {channel} not registered')
            response.status = False
        return response

def main():
    rclpy.init()

    minimal_service = MCUService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()