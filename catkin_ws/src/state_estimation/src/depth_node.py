import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float64

# Queue used for filtering
from collections import deque

# Constants
MAX_ANALOG = 1023  # in bits
MAX_ARDUINO_VOLTAGE = 5000 # in mV
RESISTANCE = 100.0 # In ohms. Set to the value of the resistor being used. Should be less than 250 ohms because 5V / 20mA (max current) = 250 omhs.
BASE_CURRENT = 4.0
CURRENT_RANGE = 16.0   # from 4.0 mA to 20.0 mA
MAX_DEPTH = 914.4      # In centimeters. 30 feet
OFFSET = 7.0          # Just substract it. May need to be recalculated when circuit is built.

# Filter window
depths = deque([0.0, 0.0, 0.0])
WINDOW_WIDTH = 3
depth = 0.0


def depthCallBack(sensor_reading):
    depth_reading = sensor_reading.data
    
    # Calculations
    current_depth = ((depth_reading * MAX_ARDUINO_VOLTAGE / MAX_ANALOG / RESISTANCE - BASE_CURRENT) / CURRENT_RANGE) * MAX_DEPTH - OFFSET

    # Apply Filters
    depths.append(current_depth)
    depth = depth + (current_depth - depths.popleft()) / WINDOW_WIDTH

    # Publish
    pub.publish(depth)

def init():
    global pub
    
    rospy.init_node('depth_node')
    rospy.Subscriber('/arduino/depth', Int16, depthCallBack)
    pub = rospy.Publisher('/state_estimation/depth', Float64)

    rospy.spin()


if __name__ == '__main__':
    try:
	init()
    except rospy.ROSInterruptException:
	pass
