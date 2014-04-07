#!/usr/bin/env python
import rospy
from controls.msg import motorCommands



def talker():
    pub = rospy.Publisher('arduino/motor', motorCommands)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        commands = motorCommands()
    	commands.cmd_surge_starboard = -100
    	commands.cmd_surge_port = -100
    	commands.cmd_sway_bow = 0
    	commands.cmd_sway_stern = 0
    	commands.cmd_heave_bow = 0
    	commands.cmd_heave_stern = 0

        #rospy.loginfo(str)
        pub.publish(commands)
        rospy.sleep(0.0097)





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
