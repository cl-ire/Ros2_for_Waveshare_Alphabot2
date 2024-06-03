#!/usr/bin/env python

'''
Controls the Waveshare Alphabot2 robot wheel motors
Updated code from DingoOz: https://github.com/DingoOz/Alphabot2_Pi_ROS/blob/master/src/alphabot2/src/driver_node
Author: Shaun Price (https://github.com/ShaunPrice)
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import time
import RPi.GPIO as GPIO

# Raspberry PI GPIO Function to pin mapping
IN1 = 13
IN2 = 12
IN3 = 21
IN4 = 20
ENA = 6
ENB = 26
PA  = 50
PB  = 50

_FREQUENCY = 22000                 


def _clip(value, minimum,maximum ):
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value


class Motor:
    def __init__(self, forward_pin, backward_pin, logger=None):
        self._forward_pin = forward_pin
        self._backward_pin = backward_pin
        
        self.logger = logger
        
        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(backward_pin, GPIO.OUT)

        self._forward_pwm = GPIO.PWM(forward_pin, _FREQUENCY)
        self._backward_pwm = GPIO.PWM(backward_pin, _FREQUENCY)

    def move(self, speed_percent):
        speed = _clip(abs(speed_percent), 0, 100)
        
        if self.logger:
            self.logger.info(f"Motor speed: {speed_percent}%, Speed: {speed}")

        # A positive speed moves wheels forward, negative moves backward
        if speed_percent >= 0:
            self._forward_pwm.start(speed)
            self._backward_pwm.start(0)
        else:
            self._backward_pwm.start(speed)
            self._forward_pwm.start(0)


class MotionDriver():
    def __init__(self, in1=13, in2=12, in3=21, in4=20, ena=6, enb=26, pa=50, pb=50, _MAX_RPM=200, logger=None):
        super().__init__()
        
        self.IN1 = in1
        self.IN2 = in2
        self.IN3 = in3
        self.IN4 = in4
        self.ENA = ena
        self.ENB = enb
        self.PA = pa
        self.PB = pb
        self._MAX_RPM = _MAX_RPM
        self.logger = logger
        

        # Configure GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
       
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        
        self.PWMA = GPIO.PWM(self.ENA, 500)
        self.PWMB = GPIO.PWM(self.ENB, 500)
        self.PWMA.start(self.PA)
        self.PWMB.start(self.PB)

        # Set ENA/ENB to high
        GPIO.output(self.ENA, GPIO.HIGH)
        GPIO.output(self.ENB, GPIO.HIGH)

        # Set forward
        self.PWMA.ChangeDutyCycle(self.PA)
        self.PWMB.ChangeDutyCycle(self.PB)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

                
        self._left_motor = Motor(self.IN1, self.IN2, self.logger)
        self._right_motor = Motor(self.IN3, self.IN4, self.logger)
        
        self.set_motor_speeds(0, 0, 0)
    
    def set_motor_speeds(self, left_speed, right_speed, duration):
        
        left_speed_percent = self.rpm_to_percent(left_speed)
        right_speed_percent = self.rpm_to_percent(right_speed)
                
        self._left_motor.move(left_speed_percent)
        self._right_motor.move(right_speed_percent)
        time.sleep(duration / 1000)
        self._left_motor.move(0)
        self._right_motor.move(0)

    def rpm_to_percent(self, rpm):
      return (rpm /self._MAX_RPM) * 100
    

    def __del__(self):
        GPIO.cleanup()


class DCMotorController(Node):
    def __init__(self):
        super().__init__('dc_motor_controller')  # Initialize the node with the name 'dc_motor_controller'
        
        self.declare_parameter('max_rpm', 140)
        self._MAX_RPM = self.get_parameter('max_rpm').value
        self.declare_parameter('motion_duration_offset', 500)
        self.duration_offset = self.get_parameter('motion_duration_offset').value
        
        # MotionDriver initzialisiern 
        self.motion_driver = MotionDriver(_MAX_RPM = self._MAX_RPM, logger = self.get_logger())  # Initialize the MotionDriver class

        self.motor_sub = self.create_subscription(  # Create a subscription to the 'motor_speeds' topic
            Int32MultiArray,
            '/motor',
            self.motor_speeds_callback,
            2
        )
        
        self.get_logger().info("DC Motor Controller Node has been started")
    
    def motor_speeds_callback(self, msg):
        mesage = msg.data
        self.get_logger().info(f"Received motor speeds: {mesage}")
        
        try:
            self.motor0_speed = mesage[0]  # Update motor0 speed
            self.motor1_speed = mesage[1]  # Update motor1 speed
            self.duration = mesage[2]      # duration update   

            # self.get_logger().info(f"Motor 0 Speed: {self.motor0_speed}")
            # self.get_logger().info(f"Motor 1 Speed: {self.motor1_speed}")
            # self.get_logger().info(f"Duration: {self.duration}")

            self.motion_driver.set_motor_speeds(self.motor0_speed, self.motor1_speed, self.duration + self.duration_offset)  # Set the motor speeds
        except IndexError as e:
            self.get_logger().error(f"Received invalid motor speeds: {e}")

    
def main():
    rclpy.init() 
    node = DCMotorController() # Create an instance of the DCMotorController class
    
    rclpy.spin(node)
    
    node.destroy_node() # Destroy the node
    rclpy.shutdown() 

if __name__ == '__main__':
    main()