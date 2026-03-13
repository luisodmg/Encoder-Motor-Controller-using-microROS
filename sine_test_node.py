import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time


class SinePublisher(Node):

    def __init__(self):

        super().__init__('sine_test_node')

        self.publisher = self.create_publisher(
            Float32,
            '/set_point',
            10
        )

        self.timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback
        )

        # parametros del seno
        self.amp = 0.5      # amplitud NORMALIZADA [-1,1]
        self.freq = 0.2     # Hz

        self.start_time = time.time()

        self.get_logger().info("Sine test started")


    def timer_callback(self):

        t = time.time() - self.start_time

        sp = self.amp * math.sin(2 * math.pi * self.freq * t)

        msg = Float32()
        msg.data = sp

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = SinePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()