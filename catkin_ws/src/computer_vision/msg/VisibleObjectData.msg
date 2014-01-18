# This custom ROS Message is what the computer vision node will be sending to
# the rest of the system. If you want to know what the ROS data type will be
# mapped to in a specific language, go to this link:
# http://wiki.ros.org/msg#Fields
# The relative angle between the robot and the object on the yaw-axis.
float64 yaw_angle

# The relative angle between the robot and the object on the pitch-axis.
float64 pitch_angle

# Distance between the robot and the object on the x-axis.
float64 x_distance

# Distance between the robot and the object on the y-axis.
float64 y_distance

# Distance between the robot and the object on the z-axis.
float64 z_distance

# The type of object that the computer vision is analysing. This variable can
# take the value of the constants defined under it.
uint8 object_type

# Constants used to identify which object is being analysed.
uint8 DOOR=0
uint8 BUOY=1
uint8 GROUND_TARGET_1=2
uint8 GROUND_TARGET_2=3
uint8 GROUND_TARGET_3=4
uint8 GROUND_TARGET_4=5
uint8 LANE=6
