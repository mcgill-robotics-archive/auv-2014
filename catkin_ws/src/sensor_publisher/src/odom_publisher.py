#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped

pub = None

def repub(odom):
	pub.publish(odom.pose)

def communicate():
	pub = rospy.Publisher('vo', Odometry)
	rospy.init_node('odom_publisher')

	sequence = 0
	odometry = Odometry()
	ang_cov = [0.0 for i in range(0, 36)]
	ang_cov[21] = ang_cov[28] = ang_cov[35] = 0.38
	large_cov = [0.1 for i in range(0, 36)]

	odometry.child_frame_id = ""

	odometry.pose.covariance = large_cov
	odometry.twist.covariance = ang_cov

	odometry.header.frame_id = "odom"
	odometry.twist.twist.angular.z = 0.1
	
	odometry.pose.pose.orientation.w = 1.0

	while not rospy.is_shutdown():
		pub.publish(odometry)
		sequence += 1

		rospy.sleep(0.01)

if __name__ == '__main__':
	try:
		communicate()
	except KeyboardInterrupt:
		sys.exit(0)
