import sys
sys.path.append('..')

import rclpy
from rclpy.node import Node

class MavSimBridge(Node):
    def __init__(self):
        super().__init__('mavsim_bridge')
        self.get_logger().info('Hi from mavsim_bridge.')


def main():
    rclpy.init()
    mavsim_bridge = MavSimBridge()
    rclpy.spin(mavsim_bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
