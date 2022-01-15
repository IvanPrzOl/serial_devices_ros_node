#!/usr/bin/env python

import rospy
import sys
import math
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField 
from std_msgs.msg import String
from SerialDataGateway import SerialDataGateway

class imu_raw_parse(object):
	def _HandleReceivedLine(self,  line):
		if (len(line) > 0 & line.startswith('$')):
			lineParts = line.split(',')
			self.Broadcast_imu_raw(lineParts)

	def Broadcast_imu_raw(self,lineParts):
		partsCount = len(lineParts)

		if (partsCount == 11): # $IMUR,timestamp,ax,ay,az,gx,gy,gz,mx,my,mz
			pass
		try:
			time_now = rospy.Time.now()
			frame_id = "imu_base"
			acc_avel_raw = Imu()
			mag_raw = MagneticField()
			acc_avel_raw.header.stamp = time_now
			acc_avel_raw.header.frame_id = frame_id
			mag_raw.header.stamp = time_now
			mag_raw.header.frame_id = frame_id
			#acc_avel_raw.arduinoTime = int(lineParts[1])
			
			# shpuld be in m/s^2
			acc_avel_raw.linear_acceleration.x = float(lineParts[2])*9.81
			acc_avel_raw.linear_acceleration.y = float(lineParts[3])*9.81
			acc_avel_raw.linear_acceleration.z = float(lineParts[4])*9.81
			# should be in rad/sec
			acc_avel_raw.angular_velocity.x = math.radians(float(lineParts[5]))
			acc_avel_raw.angular_velocity.y = math.radians(float(lineParts[6]))
			acc_avel_raw.angular_velocity.z = math.radians(float(lineParts[7]))
			# shloud be in Teslas
			# if MPU9250 is used, magnetometer orientation mismatch should be corrected to maintain x from acc and gyro as forward
			mag_raw.magnetic_field.x = float(lineParts[9])*1e-7 # x of acc corresponds to y of mag
			mag_raw.magnetic_field.y = float(lineParts[8])*1e-7 # y of acc correspond to x of mag
			mag_raw.magnetic_field.z = -float(lineParts[10])*1e-7 # z of acc is opposite to z of mag

			self._acc_vel_pub.publish(acc_avel_raw)
			self._mag_pub.publish(mag_raw)
		except:
			rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))	

	def __init__(self):
		self._acc_vel_pub = rospy.Publisher('imu/data_raw',Imu,queue_size=50)
		self._mag_pub = rospy.Publisher('imu/mag',MagneticField,queue_size=50)

		rospy.init_node('imu_raw')
		port = rospy.get_param("~port", "/dev/ttyS7")
		baudRate = int(rospy.get_param("~baudRate", 38400))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()	

if __name__ == '__main__':
	imu_raw = imu_raw_parse()
	try:
		imu_raw.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		imu_raw.Stop()

