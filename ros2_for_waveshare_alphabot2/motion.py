#!/usr/bin/env python

'''
Controls the Waveshare Alphabot2 robot wheel motors
Updated code from DingoOz: https://github.com/DingoOz/Alphabot2_Pi_ROS/blob/master/src/alphabot2/src/driver_node
Author: Shaun Price (https://github.com/ShaunPrice)
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import board
import busio
from adafruit_pca9685 import PCA9685
import time

_FREQUENCY = 100 #könnte auch 50hz oder 60hz sein wird unterschiedlich benutzt 
                 #bei der Quelle wird aber 100hz  benutzt
                 #https://docs.circuitpython.org/projects/motor/en/latest/examples.html#motor-pca9685-dc-motor
_MAX_RPM = 100  # update based on motor specifications                

class DCMotorController(Node):
    def __init__(self):
        super().__init__('dc_motor_controller')  # Initialize the node with the name 'dc_motor_controller'
        
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c)  # Initialize the PCA9685 PWM controller
            self.pca.frequency = _FREQUENCY  # Set the PWM frequency to _FREQUENCY Hz
            self.motor0 = self.pca.channels[0]  # Assign the first channel to motor0
            self.motor1 = self.pca.channels[1]  # Assign the second channel to motor1
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PCA9685: {e}")
            return

        self.motor0_speed = 0  # Initialize motor0 speed to 0
        self.motor1_speed = 0  # Initialize motor1 speed to 0

        self.motor_sub = self.create_subscription(  # Create a subscription to the 'motor_speeds' topic
            Int32MultiArray,
            '/motor',
            self.motor_speeds_callback,
            2
        )
        
        self.get_logger().info("DC Motor Controller Node has been started")
    
    def motor_speeds_callback(self, msg):
        try:
            self.motor0_speed = msg.data[0]  # Update motor0 speed
            self.motor1_speed = msg.data[1]  # Update motor1 speed

            self.get_logger().info(f"Motor 0 Speed: {self.motor0_speed}")
            self.get_logger().info(f"Motor 1 Speed: {self.motor1_speed}")

            self.set_motor_speeds(self.motor0_speed, self.motor1_speed)  # Set the motor speeds
        except IndexError as e:
            self.get_logger().error(f"Received invalid motor speeds: {e}")

    def set_motor_speeds(self, motor0_speed, motor1_speed, duration=1):
        try:
            motor0_duty_cycle = self.rpm_to_duty_cycle(motor0_speed)
            motor1_duty_cycle = self.rpm_to_duty_cycle(motor1_speed)
            
            self.motor0.duty_cycle = motor0_duty_cycle
            self.motor1.duty_cycle = motor1_duty_cycle
            time.sleep(duration)
            self.motor0.duty_cycle = 0
            self.motor1.duty_cycle = 0
        except Exception as e:
            self.get_logger().error(f"Failed to set motor speeds: {e}")

def rpm_to_duty_cycle(self, rpm):
        """Convert RPM to PCA9685 duty cycle (0-65535)."""
        if rpm < 0:
            rpm = 0
        elif rpm > _MAX_RPM:
            rpm = _MAX_RPM
 
        duty_cycle = int((rpm / _MAX_RPM) * 65535)
        return duty_cycle
#set the duty cycle based based on the rpm value

def main():
    rclpy.init() 
    node = DCMotorController() # Create an instance of the DCMotorController class
    node.destroy_node() # Destroy the node
    rclpy.shutdown() 

if __name__ == '__main__':
    main()