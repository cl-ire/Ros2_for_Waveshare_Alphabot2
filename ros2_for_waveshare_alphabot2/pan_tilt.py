#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray


class CameraPanTiltDriver(Node):

    def __init__(self):
        super().__init__('camera_pan_tilt_driver')
        self.get_logger().info("Node 'camera_pan_tilt' configuring driver")

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/Servo',
            self.listener_callback,
            2)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        servo_winkel_0 = msg.data[0]
        servo_winkel_1 = msg.data[1]

        self.get_logger().info("Servo 0 Winkel: {}".format(str(servo_winkel_0)))
        self.get_logger().info("Servo 1 Winkel: {}".format(str(servo_winkel_1)))


def main(args=None):
    rclpy.init()

    driver = CameraPanTiltDriver()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()