#!/usr/bin/env python

'''
Controls the Waveshare Alphabot2 robot wheel motors
Updated code from DingoOz: https://github.com/DingoOz/Alphabot2_Pi_ROS/blob/master/src/alphabot2/src/driver_node
Author: Shaun Price (https://github.com/ShaunPrice)
'''
import lgpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

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

def _clip(value, minimum, maximum):
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value

class Motor:
    def __init__(self, forward_pin, backward_pin):
        self._forward_pin = forward_pin
        self._backward_pin = backward_pin
        lgpio.gpio_claim_output(self._forward_pin, _FREQUENCY)
        lgpio.gpio_claim_output(self._backward_pin, _FREQUENCY)

    def move(self, speed_percent):
        speed = _clip(abs(speed_percent), 0, 100)

        # A positive speed moves wheels forward, negative moves backward
        if speed_percent >= 0:
            lgpio.pwm_set_duty_cycle(self._forward_pin, speed)
            lgpio.pwm_set_duty_cycle(self._backward_pin, 0)
        else:
            lgpio.pwm_set_duty_cycle(self._backward_pin, speed)
            lgpio.pwm_set_duty_cycle(self._forward_pin, 0)

class MotionDriver(Node):
    def __init__(self, in1=13, in2=12, in3=21, in4=20, ena=6, enb=26, pa=50, pb=50):
        super().__init__('motion_driver')
        self.loginfo("Node 'motion' configuring driver.")

        self.IN1 = in1
        self.IN2 = in2
        self.IN3 = in3
        self.IN4 = in4
        self.ENA = ena
        self.ENB = enb
        self.PA = pa
        self.PB = pb

        # Initialize the lgpio library
        self.lgpio = lgpio.gpiochip_open(0)

        # Configure GPIO
        lgpio.gpio_claim_output(self.IN1, 0)
        lgpio.gpio_claim_output(self.IN2, 0)
        lgpio.gpio_claim_output(self.IN3, 0)
        lgpio.gpio_claim_output(self.IN4, 0)
        lgpio.gpio_claim_output(self.ENA, 0)
        lgpio.gpio_claim_output(self.ENB, 0)

        self.PWMA = lgpio.pwm_start(self.ENA, _FREQUENCY, self.PA)
        self.PWMB = lgpio.pwm_start(self.ENB, _FREQUENCY, self.PB)

        # Set ENA/ENB to high
        lgpio.gpio_write(self.ENA, 1)
        lgpio.gpio_write(self.ENB, 1)

        # Set forward
        lgpio.pwm_set_duty_cycle(self.ENA, self.PA)
        lgpio.pwm_set_duty_cycle(self.ENB, self.PB)
        lgpio.gpio_write(self.IN1, 1)
        lgpio.gpio_write(self.IN2, 0)
        lgpio.gpio_write(self.IN3, 1)
        lgpio.gpio_write(self.IN4, 0)

        self.loginfo("Node 'motion' GPIO configured.")

        self._last_received = self.get_time()
        self._timeout = self.get_parameter_or('timeout', 2)
        self.rate = self.create_rate(self.get_parameter_or('rate', 10))
        self._max_speed = self.get_parameter_or('max_speed', 0.25)
        self._wheel_base = self.get_parameter_or('wheel_base', 0.093)
        self._left_motor = Motor(self.IN1, self.IN2)
        self._right_motor = Motor(self.IN3, self.IN4)
        self._left_speed_percent = 0
        self._right_speed_percent = 0

        # Setup subscriber for velocity twist message
        self.create_subscription(Twist, 'cmd_vel', self.velocity_received_callback, 1)

        # Setup subscriber for collision detection
        self.create_subscription(String, 'collision', self.collision_callback, 1)

        # Setup publisher for velocity. Used in collision to stop motors
        self.pub = self.create_publisher(Twist, 'cmd_vel', 4)

        self.loginfo("Node 'motion' configuration complete.")

    def __del__(self):
        # Close the lgpio library
        lgpio.gpiochip_close(self.lgpio)

    def velocity_received_callback(self, message):
        self._last_received = self.get_time()
        self.loginfo("Node 'motion' velocity message received")
        # Extract linear and angular velocities from the message
        linear = message.linear.x
        angular = message.angular.z

        self.loginfo("Node 'motion' linear=  " + str(linear) + ".")
        self.loginfo("Node 'motion' angular= " + str(angular) + ".")

        # Calculate wheel speeds in m/s
        left_speed = linear - angular * self._wheel_base / 2
        right_speed = linear + angular * self._wheel_base / 2

        # Ideally we'd now use the desired wheel speeds along
        # with data from wheel speed sensors to come up with the
        # power we need to apply to the wheels, but we don't have
        # wheel speed sensors. Instead, we'll simply convert m/s
        # into percent of maximum wheel speed, which gives us a
        # duty cycle that we can apply to each motor.
        self._left_speed_percent = (100 * left_speed / self._max_speed)
        self._right_speed_percent = (100 * right_speed / self._max_speed)
        self.loginfo("Node 'motion' velocity message Left= " + str(self._left_speed_percent) + "%")
        self.loginfo("Node 'motion' velocity message Right=" + str(self._right_speed_percent) + "%.")

    def run(self):
        self.loginfo("Node 'motion' running.")

        while rclpy.ok():
            # If we haven't received new commands for a while, we
            # may have lost contact with the commander-- stop moving
            delay = self.get_time() - self._last_received
            if delay < self._timeout:
                self._left_motor.move(self._left_speed_percent)
                self._right_motor.move(self._right_speed_percent)
            else:
                self._left_motor.move(0)
                self._right_motor.move(0)

            self.rate.sleep()

    def collision_callback(self, message):
        self.loginfo("Node 'motion' collision detected. Stopping")
        if message.data == 'obstacle_right' or message.data == 'obstacle_left':
            self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

def main():
    rclpy.init()
    node = MotionDriver(IN1, IN2, IN3, IN4, ENA, ENB, PA, PB)
    node.run()
    node.destroy_node()
    rclpy.shutdown()
    print("Node 'motion' Stopped")

if __name__ == '__main__':
    main()
