#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

ang = 0.1	# 10 deg/s = 0.1 deg/10ms
acc = 0.0		# Not used by now
ang_cov = 0.38	# variance
sequence = 0

def publish():
	pub = rospy.Publisher('imu_data', Imu)
	rospy.init_node('imu_publisher')

	sequence = 0
	pub_head = std_msgs.msg.Header() 

	ang_vel = geometry_msgs.msg.Vector3()
	linear_acc = geometry_msgs.msg.Vector3()
	
	ang_vel.x = ang_vel.y = linear_acc.x = linear_acc.y = linear_acc.z = 0.0
	ang_vel.z = 1.0

	quat = geometry_msgs.msg.Quaternion()
	quat.x = quat.y = quat.z = 0.0
	quat.w = 1.0

	no_cov = [-1.0 for i in range(0, 9)]
	cov_matr = [ang_cov, 0.0, 0.0, 0.0, ang_cov, 0.0, 0.0, 0.0, ang_cov]

	imu_msg = Imu()
	imu_msg.header = pub_head
	imu_msg.orientation = quat
	imu_msg.orientation_covariance = no_cov
	imu_msg.angular_velocity = ang_vel
	imu_msg.angular_velocity_covariance = cov_matr
	imu_msg.linear_acceleration = linear_acc
	imu_msg.linear_acceleration_covariance = no_cov
	
	while not rospy.is_shutdown():
		pub_head.seq = sequence
		pub_head.stamp = rospy.Time.now()
		pub_head.frame_id = "base_footprint"

		imu_msg.header = pub_head

		pub.publish(imu_msg)
		sequence += 1
		rospy.sleep(0.01)

if __name__ == '__main__':
	try:
		publish()
	except KeyboardInterrupt:
		sys.exit(0)
