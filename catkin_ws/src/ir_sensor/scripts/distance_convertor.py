#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
rospy.init_node('distance_convertor', anonymous=True)
pub = rospy.Publisher('IR/distance', Int16)
def callback(i):
    pub.publish(Int16(data=(i.data+3600)))
sub = rospy.Subscriber('IR/data' , Int16 , callback)

    
if __name__ == '__main__':
    rospy.spin()
    
