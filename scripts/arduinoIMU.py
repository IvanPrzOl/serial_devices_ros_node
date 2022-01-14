#!/usr/bin/env python

import rospy
import tf
import sys
import math
from serial_rosnode.msg import imuq
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from SerialDataGateway import SerialDataGateway

class Arduino(object):
	def _HandleReceivedLine(self,  line):
		if (len(line) > 0 & line.startswith('$')):
			lineParts = line.split(',')
			self.BroadcastIMUInfo(lineParts)
	
	def BroadcastIMUInfo(self,lineParts):
		partsCount = len(lineParts)
		if (partsCount == 6): # $IMU,timestamp,w,x,y,z
			pass
		try:
			q = Quaternion()
			norm = math.sqrt(float(lineParts[2])**2 + float(lineParts[3])**2 + float(lineParts[4])**2 + float(lineParts[5])**2)
			q.x = float(lineParts[3])/norm
			q.y = float(lineParts[4])/norm
			q.z = float(lineParts[5])/norm
			q.w = float(lineParts[2])/norm
			imu = imuq()
			imu.header.stamp = rospy.Time.now()
			imu.header.frame_id = "imu_base"
			imu.arduinoTime = int(lineParts[1])
			imu.q = q

			self._imu_pub.publish(imu)
			self._ImuTransformBroadcaster.sendTransform((0.2,0.2,0.2),(imu.q.x,imu.q.y,imu.q.z,imu.q.w),rospy.Time.now(),"imu","base_link")

		except:
			rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))

	def __init__(self):

		self._imu_pub = rospy.Publisher('imu',imuq,queue_size=50)
		self._ImuTransformBroadcaster = tf.TransformBroadcaster()

		rospy.init_node('arduinoIMU')
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
	arduino = Arduino()
	try:
		arduino.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		arduino.Stop()

