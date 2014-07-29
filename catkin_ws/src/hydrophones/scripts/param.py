#!/usr/bin/env python

# IMPORTS
import rospy

# DEFAULT PARAMETERS
BUFFERSIZE = 1024           # SIZE OF AUDIO BUFFER AND 1/2 OF FFT BUFFER
NUMBER_OF_MICS = 4          # RECEIVERS CONNECTED
SAMPLING_FREQUENCY = 192e3  # SAMPLING FREQUENCY OF SIGNAL  Hz
TARGET_FREQUENCY = 25000    # FREQUENCY OF PINGER           Hz
LENGTH_OF_PULSE = 4e-3      # LENGTH OF PING                s
DEPTH_OF_PINGER = 4.2672    # DEPTH OF PINGER FROM SURFACE  m
SPEED = 1500                # SPEED OF SOUND IN MEDIUM      m/s
HEIGHT = 0.50               # HEIGHT OF RECEIVER ARRAY      m
WIDTH = 0.32                # WIDTH OF RECEIVER ARRAY       m
LIN_TO_MIC_OFFSET = 0e-3    # LINE IN TO MIC OFFSET         s       (experimental)
THRESHOLD = 50              # THRESHOLD FOR PING            dB      (experimental)
PRACTICE = True

# SIMULATION PARAMETERS
TARGET_PINGER = (169, 54)   # TARGET PINGER COORDINATES     m
DUMMY_PINGER = (-69, 83)    # DUMMY PINGER COORDINATES      m
SNR = 20                    # SIGNAL TO NOISE RATIO         dB
SWITCHING = True            # SWITCH BETWEEN PINGERS
CONTINUOUS = False          # GENERATE CONTINUOUS SIGNAL


def get_buffersize():
    """ Returns buffersize """
    while not rospy.has_param('/hydrophones/buffersize/initial'):
        pass

    return int(rospy.get_param('/hydrophones/buffersize/initial'))


def get_final_buffersize():
    """ Returns buffersize """
    while not rospy.has_param('/hydrophones/buffersize/final'):
        pass

    return int(rospy.get_param('/hydrophones/buffersize/final'))


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


def get_mic_dimensions():
    """ Returns width and height of hydrophone array """
    while not rospy.has_param('/hydrophones/size/'):
        pass

    width = '/hydrophones/size/width'
    height = '/hydrophones/size/height'
    size = (rospy.get_param(width),rospy.get_param(height))

    return size


def get_line_in_offset():
    """ Returns LINE IN to MIC offset in seconds """
    while not rospy.has_param('/hydrophones/offset'):
        pass

    return rospy.get_param('/hydrophones/offset')


def get_threshold():
    """ Returns threshold determining ping """
    while not rospy.has_param('/hydrophones/threshold'):
        pass

    return rospy.get_param('/hydrophones/threshold')


def get_practice_pool_side_or_not():
    """ Returns TRUE if robot is in the practice pool, FALSE otherwise """
    while not rospy.has_param('/hydrophones/practice_pool'):
        pass

    return rospy.get_param('/hydrophones/practice_pool')


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


def get_switching():
    """ Returns TRUE if pinger switching is active, FALSE otherwise """
    while not rospy.has_param('/hydrophones/sim/switching'):
        pass

    return rospy.get_param('/hydrophones/sim/switching')


def get_continuous():
    """ Returns TRUE if continuous signal is active, FALSE otherwise """
    while not rospy.has_param('/hydrophones/sim/continuous'):
        pass

    return rospy.get_param('/hydrophones/sim/continuous')


def set_switching(state):
    """ TRUE to override active pinger, FALSE to alternate between target and dummy """
    rospy.set_param('/hydrophones/sim/switching',state)


def set_simulation_parameters():
    """ Creates and sets ROS simulation parameters """
    rospy.set_param('/hydrophones/sim/state',True)

    rospy.set_param('/hydrophones/sim/target/x',TARGET_PINGER[0])
    rospy.set_param('/hydrophones/sim/target/y',TARGET_PINGER[1])
    rospy.set_param('/hydrophones/sim/dummy/x',DUMMY_PINGER[0])
    rospy.set_param('/hydrophones/sim/dummy/y',DUMMY_PINGER[1])
    rospy.set_param('/hydrophones/sim/SNR',SNR)
    rospy.set_param('/hydrophones/sim/switching',SWITCHING)
    rospy.set_param('/hydrophones/sim/continuous',CONTINUOUS)


def set_parameters():
    """ Creates and sets default ROS parameters """
    rospy.set_param('/hydrophones/sim/state',False)

    rospy.set_param('/hydrophones/buffersize/initial',BUFFERSIZE)
    rospy.set_param('/hydrophones/buffersize/final',2*BUFFERSIZE)
    rospy.set_param('/hydrophones/number_of_mics',NUMBER_OF_MICS)
    rospy.set_param('/hydrophones/speed',SPEED)
    rospy.set_param('/hydrophones/fs',SAMPLING_FREQUENCY)
    rospy.set_param('/hydrophones/target',TARGET_FREQUENCY)
    rospy.set_param('/hydrophones/depth',DEPTH_OF_PINGER)
    rospy.set_param('/hydrophones/ping_length',LENGTH_OF_PULSE)
    rospy.set_param('/hydrophones/practice_pool',PRACTICE)

    rospy.set_param('/hydrophones/offset',LIN_TO_MIC_OFFSET)
    rospy.set_param('/hydrophones/threshold',THRESHOLD)

    rospy.set_param('/hydrophones/size/width',WIDTH)
    rospy.set_param('/hydrophones/size/height',HEIGHT)

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
