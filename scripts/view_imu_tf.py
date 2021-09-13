#!/usr/bin/env python   
import rospy
from serial_rosnode.msg import imuq
import tf

class view_scan(object):
    def __init__(self):
        rospy.init_node('view_imu_tf')
        self._scan_sub = rospy.Subscriber('/imu',imuq,self._broadcast_tf)
        self._ImuTransformBroadcaster = tf.TransformBroadcaster()

    def _broadcast_tf(self,data):
        
        self._ImuTransformBroadcaster.sendTransform((0.0,0.0,0.4),(data.q.x,data.q.y,data.q.z,data.q.w),rospy.Time.now(),"imu","base_link")

if __name__ == '__main__':
    view_scan_node = view_scan()
    rospy.spin()
