#!/usr/bin/env python

# IMPORTS
import rosbag
import sys
import getopt
from os import walk


def sos():
    """ Print out usage """
    print "usage: mergeBags.py OPTION"
    print "options: -a : all topics specified"
    print "         -i : imu pose and raw data"
    print "         -t : data from all temperature sensors"
    print "         -c : camera data; i.e. camera info, image rectified and color image from up and down cameras"
    print "         -d : depth sensor data"
    print "         -b : voltages from both batteries"
    print "         -e : motor message, usb, solenoid, and pressure inside main pressure vessel"
    print "         -h : all hydrophones topics (including state estimation specific topics)"
    print "         -s : all state estimation topics (including IMU specific topics)"
    print "         --help : prints this menu"


def add_imu_topics(topic_list):
    """ Add IMU topics """
    topic_list.append("state_estimation/pose")
    topic_list.append("state_estimation/raw")


def add_temperature_topics(topic_list):
    """ Add temperature topics """
    topic_list.append("status/temperature")


def add_camera_topics(topic_list):
    """ Add camera topics """
    topic_list.append("camera_down/camera_out/camera_info")
    topic_list.append("camera_down/image_rect")
    topic_list.append("camera_front_left/camera_out/camera_info")
    topic_list.append("camera_front_left/image_rect")
    topic_list.append("camera_front_right/camera_out/camera_info")
    topic_list.append("camera_front_right/image_rect")


def add_depth_topics(topic_list):
    """ Add depth topics """
    topic_list.append("electrical_interface/depth")
    topic_list.append("state_estimation/filteredDepth")


def add_battery_topics(topic_list):
    """ Add battery topics """
    topic_list.append("electrical_interface/batteryVoltage1")
    topic_list.append("electrical_interface/batteryVoltage2")


def add_electrical_interface_topics(topic_list):
    """ Add electrical interface topics """
    topic_list.append("status/usb")
    topic_list.append("electrical_interface/pressure")
    topic_list.append("electrical_interface/motor")
    topic_list.append("electrical_interface/solenoid")


def add_state_estimation_topics(topic_list):
    """ Add state estimation topics """
    add_imu_topics(topic_list)
    topic_list.append("tf")


def add_hydrophones_topics(topic_list):
    """ Add hydrophones topics """
    add_state_estimation_topics(topic_list)
    topic_list.append("hydrophones/audio")


def add_all_topics(topic_list):
    """ Add all topics """
    add_temperature_topics(topic_list)
    add_camera_topics(topic_list)
    add_depth_topics(topic_list)
    add_battery_topics(topic_list)
    add_electrical_interface_topics(topic_list)
    add_hydrophones_topics(topic_list)


def bag_we_are_looking_for(filename):
    """ Checks if the file is a bag """
    starts_correctly = filename.startswith('split_')
    ends_correctly = filename.endswith('.bag')
    not_incorrect = ('orig' not in filename) and ('active' not in filename)
    return starts_correctly and ends_correctly and not_incorrect


def get_bag_count(filename):
    """ Parses filename for correct order """
    return int(filename.split('_')[1].split('.')[0])


def get_bags(folder):
    """ Gets list of bags in folder """
    all_bags, all_files = [], []
    for (dirpath, dirnames, filenames) in walk(folder):
        all_files.extend(filenames)
        break
    for filename in all_files:
        if bag_we_are_looking_for(filename):
            all_bags.append(filename)
    return all_bags


def sort_bags(all_bags):
    """ Sorts bags in the correct order """
    bag_numbers = []
    for filename in all_bags:
        bag_numbers.append(get_bag_count(filename))
    bag_numbers.sort()
    bags_to_merge = []
    for i in bag_numbers:
        bags_to_merge.append('split_%d.bag' % i)
    return bags_to_merge


def append_bag(input, output, topic_list):
    """ Appends bag data """
    try:
        current_bag = rosbag.Bag(input, 'r')
        for topic, msg, time in current_bag.read_messages(topics=topic_list):
            output.write(topic, msg, time)
    finally:
        current_bag.close()


def merge_bags(bags_to_merge, output, topic_list, folder):
    """ Merges all bags into one """
    number_of_bags = len(bags_to_merge)
    print "%d bags to merge" % (number_of_bags)
    counter = 1
    for bag in bags_to_merge:
        print "Merging %d of %d..." % (counter, number_of_bags)
        append_bag(folder + bag, output, topic_list)
        counter += 1


def main(folder, name, topic_list):
    """ Set up and merge topics from bags in folder """
    output_bag = rosbag.Bag(folder + name, 'w')
    all_bags = get_bags(folder)
    bags_to_merge = sort_bags(all_bags)
    if len(bags_to_merge) >= 1:
        try:
            merge_bags(bags_to_merge, output_bag, topic_list, folder)
        finally:
            output_bag.close()
        print "Merged bag saved to %s%s" % (folder, name)
    else:
        print "No bags to merge."
        sys.exit(1)


def get_arguments(args):
    """ Parse arguments """
    folder = "."
    name = "merge"
    topic_list = []

    if len(args) == 0:
        sos()
        sys.exit(1)

    try:
        opts, args = getopt.getopt(args,"aitcdbehsf:",["help=","folder="])
    except getopt.GetoptError:
        sos()
        sys.exit(2)

    for opt, arg in opts:
        if opt == "--help":
            sos()
            sys.exit(1)
        elif opt == "-a":
            print "Subscribing to all topics..."
            add_all_topics(topic_list)
            name += "_all"
        elif opt == "-i":
            print "Subscribing to IMU topics..."
            add_imu_topics(topic_list)
            name += "_imu"
        elif opt == "-t":
            print "Subscribing to temperature topics..."
            add_temperature_topics(topic_list)
            name += "_temp"
        elif opt == "-c":
            print "Subscribing to camera topics..."
            add_camera_topics(topic_list)
            name += "_cam"
        elif opt == "-d":
            print "Subscribing to depth topics..."
            add_depth_topics(topic_list)
            name += "_depth"
        elif opt == "-b":
            print "Subscribing to battery voltages topics..."
            add_battery_topics(topic_list)
            name += "_batt"
        elif opt == "-e":
            print "Subscribing to electrical interface topics..."
            add_electrical_interface_topics(topic_list)
            name += "_elec"
        elif opt == "-h":
            print "Subscribing to hydrophones and state estimation topics..."
            add_hydrophones_topics(topic_list)
            name += "_hydro"
        elif opt == "-s":
            print "Subscribing to state estimation and IMU topics..."
            add_state_estimation_topics(topic_list)
            name += "_state"
        elif opt in ("-f", "--folder"):
            if arg.endswith('/'):
                folder = arg
            else:
                folder = arg + "/"

    name += ".bag"

    return folder, name, topic_list


if __name__ == '__main__':
    folder, name, topic_list = get_arguments(sys.argv[1:])
    main(folder, name, topic_list)
