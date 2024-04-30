#!/usr/bin/env python

'''
Controls the Waveshare Alphabot2 robot wheel motors
Updated code from DingoOz: https://github.com/DingoOz/Alphabot2_Pi_ROS/blob/master/src/alphabot2/src/driver_node
Author: Shaun Price (https://github.com/ShaunPrice)
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import adafruit_pca9685
import RPi.GPIO as GPIO
import time

_FREQUENCY = 22000

class DCMotorController(Node):
    def __init__(self):
        super().__init__('dc_motor_controller') # Initialize the node with the name 'dc_motor_controller'
        self.pca = adafruit_pca9685.PCA9685(busio.I2C(board.SCL, board.SDA)) # Initialize the PCA9685 PWM controller
        self.pca.frequency = 50 # Set the PWM frequency to 50 Hz
        self.motor1 = self.pca.channels[0] # Assign the first channel to motor1
        self.motor2 = self.pca.channels[1] # Assign the second channel to motor2
        self.motor1_speed = 0 # Initialize motor1 speed to 0
        self.motor2_speed = 0 # Initialize motor2 speed to 0
        self.motor_sub = self.create_subscription( # Create a subscription to the 'motor_speeds' topic
            Int32MultiArray,
            'motor_speeds',
            self.motor_speeds_callback,
            10
        )

    def motor_speeds_callback(self, msg):
        self.motor1_speed = msg.data[0] # Update motor1 speed
        self.motor2_speed = msg.data[1] # Update motor2 speed
        self.set_motor_speeds(self.motor1_speed, self.motor2_speed) # Set the motor speeds

    def set_motor_speeds(self, motor1_speed, motor2_speed):
        self.motor1.duty_cycle = motor1_speed # Set the duty cycle for motor1
        self.motor2.duty_cycle = motor2_speed # Set the duty cycle for motor2

def main():
    rclpy.init() # Initialize ROS2
    node = DCMotorController() # Create an instance of the DCMotorController class
    node.destroy_node() # Destroy the node
    rclpy.shutdown() # Shutdown ROS2
    print("Node 'motion' Stopped")

if __name__ == '__main__':
    main()