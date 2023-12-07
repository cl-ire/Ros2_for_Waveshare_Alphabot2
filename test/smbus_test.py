#!/usr/bin/env python3

import time
import math
import smbus


# Camera Pan
# Camera Tilt


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

	# def __del__(self):
	# 	self.write(self.__MODE1, 0x00)


def main(args=None):
	# channel 0 = pan 
	# channel 1 = tilt
	offset = [0, 0]		
	limit_left = [-90, -80]
	limit_right = [90, 45]
	
	pwm = PCA9685(offset, limit_left, limit_right, 0x40)
	pwm.setPWMFreq(50)
	
	pwm.setServoPulse(0, 0)
	pwm.setServoPulse(1, 0)
	
	time.sleep(5)

	pwm.setServoPulse(0, 45)
	pwm.setServoPulse(1, 45)

	time.sleep(5)

	pwm.setServoPulse(0, -45)
	pwm.setServoPulse(1, -45)

	time.sleep(5)

	pwm.setServoPulse(0, 90)
	pwm.setServoPulse(1, 90)

	time.sleep(5)

	pwm.setServoPulse(0, 0)
	pwm.setServoPulse(1, 0)
	

if __name__ == '__main__':
	main()
