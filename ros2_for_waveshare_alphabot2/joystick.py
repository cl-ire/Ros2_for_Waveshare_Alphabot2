#!/usr/bin/env python

import RPi.GPIO as GPIO
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

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.CTR, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.A, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.B, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.C, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.D, GPIO.IN, GPIO.PUD_UP)

        self.rate = self.create_rate(self.get_parameter_or('~rate', 10).value)

        # Setup publisher for joystick
        self.pub = self.create_publisher(String, 'joystick', 4)
        self.get_logger().info("Node 'joystick' configured.")

    def __del__(self):
        GPIO.cleanup()

    def run(self):
        self.get_logger().info("Node 'joystick' running.")
        while rclpy.ok():
            if GPIO.input(self.CTR) == 0:
                # center
                while GPIO.input(self.CTR) == 0:
                    self.pub.publish(String("Center"))
                    self.get_logger().info("Node 'joystick' Center.")
            elif GPIO.input(self.A) == 0:
                # up
                while GPIO.input(self.A) == 0:
                    self.pub.publish(String("Up"))
                    self.get_logger().info("Node 'joystick' Up.")
            elif GPIO.input(self.B) == 0:
                # right
                while GPIO.input(self.B) == 0:
                    self.pub.publish(String("Right"))
                    self.get_logger().info("Node 'joystick' Right.")
            elif GPIO.input(self.C) == 0:
                # left
                while GPIO.input(self.C) == 0:
                    self.pub.publish(String("Left"))
                    self.get_logger().info("Node 'joystick' Left.")
            elif GPIO.input(self.D) == 0:
                # down
                while GPIO.input(self.D) == 0:
                    self.pub.publish(String("Down"))
                    self.get_logger().info("Node 'joystick' Down.")

            self.rate.sleep()

def main():
    rclpy.init()
    try:
        rclpy.spin(JoystickDriver())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
