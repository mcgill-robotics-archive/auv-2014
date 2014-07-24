#!/usr/bin/env python

# TODO: DISTINGUISH PRACTICE PARAMETERS FROM COMPETITION

# IMPORTS
import rospy

# DEFAULT PARAMETERS
BUFFERSIZE = 4096           # SIZE OF FFT BUFFER
NUMBER_OF_MICS = 4          # RECEIVERS CONNECTED
SAMPLING_FREQUENCY = 192e3  # SAMPLING FREQUENCY OF SIGNAL  Hz
TARGET_FREQUENCY = 30000    # FREQUENCY OF PINGER           Hz
LENGTH_OF_PULSE = 1.3e-3    # LENGTH OF PING                s
DEPTH_OF_PINGER = 4.2672    # DEPTH OF PINGER FROM SURFACE  m
SPEED = 1500                # SPEED OF SOUND IN MEDIUM      m/s
HEIGHT = 1.83               # HEIGHT OF RECEIVER ARRAY      m
WIDTH = 0.91                # WIDTH OF RECEIVER ARRAY       m

# SIMULATION PARAMETERS
LINEAR_CHIRP = False        # TYPE OF SIGNAL TO CREATE
TARGET_PINGER = (169, 54)   # TARGET PINGER COORDINATES     m
DUMMY_PINGER = (-69, 83)    # DUMMY PINGER COORDINATES      m
SNR = 20                    # SIGNAL TO NOISE RATIO         dB


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


def get_pulse_length():
    """ Returns length of ping in s """
    while not rospy.has_param('/hydrophones/ping_length'):
        pass

    return rospy.get_param('/hydrophones/ping_length')


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


def get_practice_pool_side_or_not():
    """ Returns True if robot is in the practice pool, false otherwise """
    while not rospy.has_param('/hydrophones/practice_pool'):
        pass

    return rospy.get_param('/hydrophones/practice_pool')


def get_linear_chirp_or_not():
    """ Returns True if linear chirp, false for pure sine wave """
    while not rospy.has_param('/hydrophones/linear_chirp'):
        pass

    return rospy.get_param('/hydrophones/linear_chirp')


def get_simulation_target():
    """ Returns (x,y) coordinates of the simulated target pinger """
    while not rospy.has_param('/hydrophones/sim/target'):
        pass

    x = rospy.get_param('/hydrophones/sim/target/x')
    y = rospy.get_param('/hydrophones/sim/target/y')

    return (x,y)


def get_simulation_dummy():
    """ Returns (x,y) coordinates of the simulated dummy pinger """
    while not rospy.has_param('/hydrophones/sim/dummy'):
        pass

    x = rospy.get_param('/hydrophones/sim/dummy/x')
    y = rospy.get_param('/hydrophones/sim/dummy/y')

    return (x,y)


def get_snr():
    """ Returns signal-to-noise ratio """
    while not rospy.has_param('/hydrophones/sim/SNR'):
        pass

    return rospy.get_param('/hydrophones/sim/SNR')


def get_simulation_state():
    """ Returns state of simulation """
    while not rospy.has_param('/hydrophones/sim/state'):
        pass

    return rospy.get_param('/hydrophones/sim/state')


def set_simulation_parameters():
    """ Creates and sets ROS simulation parameters """
    rospy.set_param('/hydrophones/sim/state',True)

    rospy.set_param('/hydrophones/sim/target/x',TARGET_PINGER[0])
    rospy.set_param('/hydrophones/sim/target/y',TARGET_PINGER[1])
    rospy.set_param('/hydrophones/sim/dummy/x',DUMMY_PINGER[0])
    rospy.set_param('/hydrophones/sim/dummy/y',DUMMY_PINGER[1])
    rospy.set_param('/hydrophones/sim/SNR',SNR)
    rospy.set_param('/hydrophones/linear_chirp', LINEAR_CHIRP)


def set_parameters():
    """ Creates and sets default ROS parameters """
    rospy.set_param('/hydrophones/sim/state',False)

    rospy.set_param('/hydrophones/buffersize',BUFFERSIZE)
    rospy.set_param('/hydrophones/number_of_mics',NUMBER_OF_MICS)
    rospy.set_param('/hydrophones/speed',SPEED)
    rospy.set_param('/hydrophones/fs',SAMPLING_FREQUENCY)
    rospy.set_param('/hydrophones/target',TARGET_FREQUENCY)
    rospy.set_param('/hydrophones/depth',DEPTH_OF_PINGER)
    rospy.set_param('/hydrophones/ping_length',LENGTH_OF_PULSE)

    rospy.set_param('/hydrophones/pos/0/x',0)
    rospy.set_param('/hydrophones/pos/0/y',0)

    rospy.set_param('/hydrophones/pos/1/x',WIDTH)
    rospy.set_param('/hydrophones/pos/1/y',0)

    rospy.set_param('/hydrophones/pos/2/x',WIDTH)
    rospy.set_param('/hydrophones/pos/2/y',HEIGHT)

    rospy.set_param('/hydrophones/pos/3/x',0)
    rospy.set_param('/hydrophones/pos/3/y',HEIGHT)

    rospy.logwarn('PARAMETERS WERE SET')


if __name__ == '__main__':
    try:
        rospy.init_node('param')
        set_parameters()
    except:
        print 'ROS NOT RUNNING'
        exit(1)
