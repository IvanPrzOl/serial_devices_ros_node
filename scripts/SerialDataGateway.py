#!/usr/bin/env python

import threading
import serial
from cStringIO import StringIO
import time
import rospy
import sys
import binascii

def _OnLineReceived(line):
	print(line)

class SerialDataGateway(object):
	'''
	Helper class for receiving lines from a serial port
	'''

	def __init__(self, port="/dev/ttyS13", baudrate=38400, lineHandler = _OnLineReceived):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		receivedLineHandler: The function to call when a line was received.
		'''
		self._Port = port
		self._Baudrate = baudrate
		self.ReceivedLineHandler = lineHandler
		self._KeepRunning = False
		self._bytesReceived = 0

	def Start(self):
		try:
		    self._Serial = serial.Serial(port = self._Port, baudrate = self._Baudrate, bytesize = serial.EIGHTBITS)
		    self._Serial.flush()
		    self._Serial.reset_input_buffer
		    self._KeepRunning = True
		    self._ReceiverThread = threading.Thread(target=self._Listen)
		    self._ReceiverThread.setDaemon(True)
		    self._ReceiverThread.start()
		except:
			print("Port not found")

	def Stop(self):
		if self._KeepRunning:
			rospy.loginfo("Stopping serial gateway")
			self._KeepRunning = False
			time.sleep(.1)
			self._Serial.close()

	def _Listen(self):
		stringIO = StringIO()
		while self._KeepRunning:
			data = self._Serial.read(1) # reads one character at time
			#data = data.decode("ASCII")
			if data == '\r' or data == '':
				pass
			if data == '\n' and self._bytesReceived > 0:
				self.ReceivedLineHandler(stringIO.getvalue())
				stringIO.close()
				stringIO = StringIO()
				self._bytesReceived = 0
			if data == '\n' and self._bytesReceived == 0:
				stringIO.close()
				stringIO = StringIO()
			else:
				stringIO.write(data)
				self._bytesReceived += 1

	def Write(self, data):
		info = "Writing to serial port: %s" %data
		rospy.loginfo(info)
		self._Serial.write(data)

if __name__ == '__main__':
	dataReceiver = SerialDataGateway("/dev/ttyS16",  38400)
	dataReceiver.Start()
	raw_input("Hit <Enter> to end.")
	dataReceiver.Stop()
