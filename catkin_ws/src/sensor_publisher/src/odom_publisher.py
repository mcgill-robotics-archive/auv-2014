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
	rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, repub)

	sequence = 0
	odometry = Odometry()
	no_cov = [0.0 for i in range(0, 36)]
	no_cov[0] = -1.0

	odometry.pose.covariance = no_cov
	odometry.twist.covariance = no_cov

	odometry.header.frame_id = "0"
	odometry.twist.twist.angular.z = 0.1
	
	pub.publish(odometry)

	rospy.spin()


if __name__ == '__main__':
	try:
		communicate()
	except KeyboardInterrupt:
		sys.exit(0)
