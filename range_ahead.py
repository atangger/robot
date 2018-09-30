#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi
def scan_callback(msg):	
	range_ahead = msg.ranges[len(msg.ranges)/2]
	print "range ahead: %0.1f" % range_ahead

class bug2():
	def __init__(self):
		rospy.init_node('range_ahead',log_level=rospy.DEBUG)
		self.tf_listener = tf.TransformListener()
		self.world_frame = '/odom'
		self.base_frame = '/base_link'
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

		# scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
		# try:
		# 	self.tf_listener.waitForTransform(self.world_frame,self.base_frame, rospy.Time(), rospy.Duration(1.0))
		# except Exception as e:
		# 	print e.message
		print 'get here'
		forward = Twist()
		forward.linear.x = 0.5

		while not rospy.is_shutdown():
			self.get_odom()
			self.cmd_vel_pub.publish(forward)
			rospy.sleep(2)
		rospy.spin()



	def get_odom(self):
		try:
			self.tf_listener.waitForTransform(self.world_frame,self.base_frame, rospy.Time(), rospy.Duration(4.0))
			(trans, rot) = self.tf_listener.lookupTransform(self.world_frame,self.base_frame,rospy.Time(0))
			print 'now tans = '
			print trans
		except (tf.Exception, tf.ConnectivityException, tf.LookupException) as e:
			rospy.loginfo(e.message)
			rospy.loginfo("TF Exception")
			return 

if __name__ == '__main__':
	try:
		bug2()
	except:
		rospy.loginfo("bug2 node terminated")
