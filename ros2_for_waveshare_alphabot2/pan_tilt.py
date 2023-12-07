#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray

import time
import math
import smbus


class PCA9685:      # https://www.kampis-elektroecke.de/raspberry-pi/raspberry-pi-i2c/servo/
		
	def __init__(self, offset, limit_left, limit_right, address=0x40, debug=False):
		self.offset = offset		
		self.limit_left = limit_left
		self.limit_right = limit_right

		self.bus = smbus.SMBus(1)
		self.address = address
		self.debug = debug
		self.write(self.__MODE1, 0x00)
	
	# Registers/etc.
	__SUBADR1		= 0x02
	__SUBADR2		= 0x03
	__SUBADR3		= 0x04
	__MODE1			= 0x00
	__PRESCALE		= 0xFE
	__LED0_ON_L		= 0x06
	__LED0_ON_H		= 0x07
	__LED0_OFF_L	= 0x08
	__LED0_OFF_H	= 0x09
	__ALLLED_ON_L	= 0xFA
	__ALLLED_ON_H	= 0xFB
	__ALLLED_OFF_L	= 0xFC
	__ALLLED_OFF_H	= 0xFD

	def write(self, reg, value):
		self.bus.write_byte_data(self.address, reg, value)
			
	def read(self, reg):
		# Read an unsigned byte from the I2C device
		result = self.bus.read_byte_data(self.address, reg)
		return result
		
	def setPWMFreq(self, freq):
		# Sets the PWM frequency
		# Calculate the prescaler 
		prescaleval = 25000000.0		# Oszilator Frequenz des PCA9685 controlers 25MHz
		prescaleval /= 4096.0			# 12-bit 
		prescaleval /= float(freq)      # Sendefrequenz 
		prescaleval -= 1.0
		prescale = math.floor(prescaleval + 0.5) 

		# write cofiguration registry 
		oldmode = self.read(self.__MODE1)               # read the old cofiguration bit 
		newmode = (oldmode & 0x7F) | 0x10				# calcualte the new configuration bit 
		self.write(self.__MODE1, newmode)				# write the configuration bit
		self.write(self.__PRESCALE, int(prescale))      # write into PRE_SCALE-Register
		self.write(self.__MODE1, oldmode)               
		time.sleep(0.005)
		self.write(self.__MODE1, oldmode | 0x80)

	def setPWM(self, channel, on, off):
		# Sets a single PWM channel
		self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
		self.write(self.__LED0_ON_H+4*channel, on >> 8)
		self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
		self.write(self.__LED0_OFF_H+4*channel, off >> 8)
			
	def setServoPulse(self, channel, angle):
		# Sets the Servo Pulse,The PWM frequency must be 50HZ
		if self.limit_left[channel] >= angle:
			angle = self.limit_left[channel] + self.offset[channel]
		elif self.limit_right[channel] <= angle:
			angle = self.limit_right[channel] + self.offset[channel]
		else:
			angle = angle + self.offset[channel]

		
		angle = 7.5 - ((angle) * 0.055)
		pulse = int(4096.0 / 100.0 * angle)
		self.setPWM(channel, 0, int(pulse))


class CameraPanTiltDriver(Node):

	def __init__(self):
		super().__init__('camera_pan_tilt_driver')
		self.get_logger().info("Node 'camera_pan_tilt' configuring driver")

		self.setup_servo()

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

		self.pwm.setServoPulse(0, servo_winkel_0)
		self.pwm.setServoPulse(1, servo_winkel_1)
	
		time.sleep(0.2)
	
	def setup_servo(self):
		# channel 0 = pan 
		# channel 1 = tilt
		offset = [30, 0]		
		limit_left = [-90, -80]
		limit_right = [90, 45]
	
		self.pwm = PCA9685(offset, limit_left, limit_right, 0x40)
		self.pwm.setPWMFreq(50)
	
		self.pwm.setServoPulse(0, 0)
		self.pwm.setServoPulse(1, 0)
	
		time.sleep(5)


def main(args=None):
	rclpy.init()

	driver = CameraPanTiltDriver()

	rclpy.spin(driver)

	driver.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()