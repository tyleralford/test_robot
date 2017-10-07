#!/usr/bin/env python

from sensor_msgs.msg import Imu
import roslib
import rospy
import smbus
import math

print "Starting node..."

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

print "Waking up IMU"

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

print "Success..."

def publisher():
	pub = rospy.Publisher('imu_data', Imu, queue_size=1)
	print "Publisher('imu_data', IMU, queue_size=1)"
	print "Initializing node..."
	rospy.init_node('mpu6050', anonymous=True)
	print "Success!"
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		#print "putting together msg"
		msg = Imu()
		#header
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = '0'
		msg.header.seq = 0
		
		#covariance estimations

		# Orientation covariance estimation:
		# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
		# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
		# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
		# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
		# i.e. variance in yaw: 0.0025
		# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
		# static roll/pitch error of 0.8%, owing to gravity orientation sensing
		# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
		# so set all covariances the same.
		msg.orientation_covariance = [
		0.0025 , 0 , 0,
		0, 0.0025, 0,
		0, 0, 0.0025
		]

		#stand-in orientation values
		msg.orientation.x = 0
		msg.orientation.y = 0
		msg.orientation.z = 0
		msg.orientation.w = 0
		
		# Angular velocity covariance estimation:
		# Observed gyro noise: 4 counts => 0.28 degrees/sec
		# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
		# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
		msg.angular_velocity_covariance = [
		0.02, 0 , 0,
		0 , 0.02, 0,
		0 , 0 , 0.02
		]
		
		#read gyro values
		msg.angular_velocity.x = read_word_2c(0x43)/131
		msg.angular_velocity.y = read_word_2c(0x45)/131
		msg.angular_velocity.z = read_word_2c(0x47)/131
	
		# linear acceleration covariance estimation:
		# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
		# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
		# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
		msg.linear_acceleration_covariance = [
		0.04 , 0 , 0,
		0 , 0.04, 0,
		0 , 0 , 0.04
		]

		#read accel values
		msg.linear_acceleration.x = read_word_2c(0x3b) / (16384.0 / 9.807)
		msg.linear_acceleration.y = read_word_2c(0x3d) / (16384.0 / 9.807)
		msg.linear_acceleration.z = read_word_2c(0x3f) / (16384.0 / 9.807)
		#print "publishing message"
		pub.publish(msg)
		#print "sucess"
		#rospy.spin()
		#print "spinning"
		rate.sleep()
		#print "rate.sleep"

if __name__ == '__main__':
	try:
		print "running publisher"
		publisher()
	except rospy.ROSInterruptException:
		pass
