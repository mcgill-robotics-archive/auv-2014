#!/usr/bin/env python
import rospy
from controls.msg import motorCommands



def talker():
    pub = rospy.Publisher('arduino/motor', motorCommands)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        commands = motorCommands()
	commands.cmd_x1 = -100
	commands.cmd_x2 = -100
	commands.cmd_y1 = 0
	commands.cmd_y2 = 0
	commands.cmd_z1 = 0
	commands.cmd_z2 = 0
        #rospy.loginfo(str)
        pub.publish(commands)
        rospy.sleep(0.0097)





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
