#!/usr/bin/env python

import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Joystick
CTR = 7  # Center
A = 8    # Up
B = 9    # Right
C = 10   # Left
D = 11   # Down

class JoystickDriver(Node):
    def __init__(self, ctr=7, a=8, b=9, c=10, d=11):
        super().__init__('joystick_driver')
        self.get_logger().info("Node 'joystick' configuring driver")

        self.CTR = ctr
        self.A = a
        self.B = b
        self.C = c
        self.D = d

        # Initialize the lgpio library
        self.lgpio = lgpio.gpiochip_open(0)
        
        # Set the GPIO pins as inputs with pull-up resistors
        lgpio.gpio_claim_input(self.lgpio, self.CTR, lgpio.PI_PUD_DOWN)
        lgpio.gpio_claim_input(self.lgpio, self.A, lgpio.PI_PUD_DOWN)
        lgpio.gpio_claim_input(self.lgpio, self.B, lgpio.PI_PUD_DOWN)
        lgpio.gpio_claim_input(self.lgpio, self.C, lgpio.PI_PUD_DOWN)
        lgpio.gpio_claim_input(self.lgpio, self.D, lgpio.PI_PUD_DOWN)

        self.rate = self.create_rate(self.get_parameter_or('~rate', 10))

        # Setup publisher for joystick
        self.pub = self.create_publisher(String, 'joystick', 4)
        self.get_logger().info("Node 'joystick' configured.")

    def __del__(self):
        # Close the lgpio library
        lgpio.gpiochip_close(self.lgpio)

    def run(self):
        self.get_logger().info("Node 'joystick' running.")
        while rclpy.ok():
            if lgpio.gpio_read(self.lgpio, self.CTR) == 0:
                # center
                while lgpio.gpio_read(self.lgpio, self.CTR) == 0:
                    self.pub.publish(String("Center"))
                    self.get_logger().info("Node 'joystick' Center.")
            elif lgpio.gpio_read(self.lgpio, self.A) == 0:
                # up
                while lgpio.gpio_read(self.lgpio, self.A) == 0:
                    self.pub.publish(String("Up"))
                    self.get_logger().info("Node 'joystick' Up.")
            elif lgpio.gpio_read(self.lgpio, self.B) == 0:
                # right
                while lgpio.gpio_read(self.lgpio, self.B) == 0:
                    self.pub.publish(String("Right"))
                    self.get_logger().info("Node 'joystick' Right.")
            elif lgpio.gpio_read(self.lgpio, self.C) == 0:
                # left
                while lgpio.gpio_read(self.lgpio, self.C) == 0:
                    self.pub.publish(String("Left"))
                    self.get_logger().info("Node 'joystick' Left.")
            elif lgpio.gpio_read(self.lgpio, self.D) == 0:
                # down
                while lgpio.gpio_read(self.lgpio, self.D) == 0:
                    self.pub.publish(String("Down"))
                    self.get_logger().info("Node 'joystick' Down.")

            self.rate.sleep()

def main():
    
    rclpy.init()
    node = JoystickDriver()
    try:
        node.run()
    finally:
        node.__del__()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
