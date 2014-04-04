import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float64

def depthCallBack(sensor_reading):
    depth_reading = sensor_reading.data
    
    # Calculations
    current_depth = ((depth_reading * max_arduino_voltage / max_analog / resistance - base_current) / current_range) * max_depth - offset
    current_depth = current_depth / 100.0	# To meters

    # Publish
    pub.publish(current_depth)

def init():
    global pub
    global max_analog
    global max_arduino_voltage
    global resistance
    global base_current
    global current_range
    global max_depth
    global offset
    
    
    rospy.init_node('depth_node')
    rospy.Subscriber('/arduino/depth', Int16, depthCallBack)
    pub = rospy.Publisher('/state_estimation/depth', Float64)
    
	# Constants
	max_analog = 1023  # in bits
	max_arduino_voltage = 5000 # in mV
	resistance = 100.0 # In ohms. Set to the value of the resistor being used. Should be less than 250 ohms because 5V / 20mA (max current) = 250 omhs.
	base_current = 4.0
	current_range = 16.0   # from 4.0 mA to 20.0 mA
	max_depth = 914.4      # In centimeters. 30 feet
	offset = 7.0          # Just substract it. May need to be recalculated when circuit is built.

    rospy.spin()


if __name__ == '__main__':
    try:
	init()
    except rospy.ROSInterruptException:
	pass
