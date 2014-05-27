#!/usr/bin/env python

# IMPORTS
import rospy

# PARAMETERS
BUFFERSIZE = 128            # SIZE OF FFT BUFFER
NUMBER_OF_MICS = 4          # RECEIVERS CONNECTED
SAMPLING_FREQUENCY = 192e3  # SAMPLING FREQUENCY OF SIGNAL  Hz
TARGET_FREQUENCY = 1000     # FREQUENCY OF PINGER           Hz
DEPTH_OF_PINGER = 4.2672    # DEPTH OF PINGER FROM SURFACE  m
SPEED = 1500                # SPEED OF SOUND IN MEDIUM      m/s
HEIGHT = 1.83               # HEIGHT OF RECEIVER ARRAY      m
WIDTH = 0.91                # WIDTH OF RECEIVER ARRAY       m
PINGER_X = 169.2
PINGER_Y = 54.8


def get_buffersize():
    """ Returns buffersize """
    while not rospy.has_param('/hydrophones/buffersize'):
        pass

    return int(rospy.get_param('/hydrophones/buffersize'))


def get_number_of_mics():
    """ Returns number of hydrophones in array """
    while not rospy.has_param('/hydrophones/number_of_mics'):
        pass

    return int(rospy.get_param('/hydrophones/number_of_mics'))


def get_speed():
    """ Returns speed of sound in medium in m/s """
    while not rospy.has_param('/hydrophones/speed'):
        pass

    return rospy.get_param('/hydrophones/speed')


def get_sampling_frequency():
    """ Returns sampling frequency in Hz """
    while not rospy.has_param('/hydrophones/fs'):
        pass

    return int(rospy.get_param('/hydrophones/fs'))


def get_target_frequency():
    """ Returns target frequency in Hz """
    while not rospy.has_param('/hydrophones/target'):
        pass

    return int(rospy.get_param('/hydrophones/target'))


def get_depth_of_pinger():
    """ Returns depth of the pinger from the surface in m """
    while not rospy.has_param('/hydrophones/depth'):
        pass

    return rospy.get_param('/hydrophones/depth')


def get_mic_positions():
    """ Returns (x,y) coordinates of every hydrophone """
    while not rospy.has_param('/hydrophones/pos/'):
        pass

    pos = [() for i in range(NUMBER_OF_MICS)]
    for i in range(NUMBER_OF_MICS):
        x = '/hydrophones/pos/' + str(i) + '/x'
        y = '/hydrophones/pos/' + str(i) + '/y'
        pos[i] = (rospy.get_param(x),rospy.get_param(y))

    return pos


def get_simulation_solution():
    """ Returns (x,y) coordinates of the simulated pinger """
    while not rospy.has_param('/hydrophones/simulation'):
        pass

    x = rospy.get_param('/hydrophones/simulation/x')
    y = rospy.get_param('/hydrophones/simulation/y')

    return (x,y)


def set_simulation_parameters():
    """ Creates and sets ROS simulation parameters """
    rospy.set_param('/hydrophones/simulation/x',PINGER_X)
    rospy.set_param('/hydrophones/simulation/y',PINGER_Y)


def set_parameters():
    """ Creates and sets default ROS parameters """
    rospy.set_param('/hydrophones/buffersize',BUFFERSIZE)
    rospy.set_param('/hydrophones/number_of_mics',NUMBER_OF_MICS)
    rospy.set_param('/hydrophones/speed',SPEED)
    rospy.set_param('/hydrophones/fs',SAMPLING_FREQUENCY)
    rospy.set_param('/hydrophones/target',TARGET_FREQUENCY)
    rospy.set_param('/hydrophones/depth',DEPTH_OF_PINGER)

    rospy.set_param('/hydrophones/pos/0/x',0)
    rospy.set_param('/hydrophones/pos/0/y',0)

    rospy.set_param('/hydrophones/pos/1/x',WIDTH)
    rospy.set_param('/hydrophones/pos/1/y',0)

    rospy.set_param('/hydrophones/pos/2/x',WIDTH)
    rospy.set_param('/hydrophones/pos/2/y',HEIGHT)

    rospy.set_param('/hydrophones/pos/3/x',0)
    rospy.set_param('/hydrophones/pos/3/y',HEIGHT)


if __name__ == '__main__':
    set_parameters()
