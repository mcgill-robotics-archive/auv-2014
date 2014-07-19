#!/usr/bin/env python

# IMPORTS
import rosbag
import sys
import getopt
import time
from os import path, popen, walk

# PARSING
prefix = "split"
delimiter = "_"
extension = ".bag"


class colors:
    """ Color codes for printing """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'


def format_progress(first, second, delimiter):
    """ Formats string correctly """
    rows, columns = popen('stty size', 'r').read().split()
    length_of_delimiter = int(columns) - len(first)
    return first + second.rjust(length_of_delimiter,delimiter)



def sizeof_file(size):
    """ Gets size of bag in human readable format """
    for x in ['bytes','KB','MB','GB','TB']:
        if size < 1024.0:
            return "%3.1f%s" % (size, x)
        size /= 1024.0


def sos():
    """ Print out usage """
    print colors.FAIL + "usage: mergeBags.py OPTION" + colors.ENDC
    print colors.WARNING + "options: -a : all topics specified"
    print "         -i : imu pose and raw data"
    print "         -t : data from all temperature sensors"
    print "         -c : camera data; i.e. camera info, image rectified and color image from up and down cameras"
    print "         -d : depth sensor data"
    print "         -b : voltages from both batteries"
    print "         -e : motor message, usb, solenoid, and pressure inside main pressure vessel"
    print "         -h : all hydrophones topics (including state estimation specific topics)"
    print "         -s : all state estimation topics (including IMU specific topics)"
    print "         --help : prints this menu" + colors.ENDC


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
    starts_correctly = filename.startswith(prefix + delimiter)
    ends_correctly = filename.endswith(extension)
    not_incorrect = ("orig" not in filename) and ("active" not in filename)
    return starts_correctly and ends_correctly and not_incorrect


def get_bag_count(filename):
    """ Parses filename for correct order """
    return int(filename.split(delimiter)[1].split(".")[0])


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
        bags_to_merge.append("%s%s%d%s" % (prefix, delimiter, i, extension))
    return bags_to_merge


def append_bag(input, output, topic_list):
    """ Appends bag data """
    try:
        current_bag = rosbag.Bag(input, "r")
        progress = "Merging %d of %d" % (get_bag_count(path.basename(input)) + 1, number_of_bags)
        print format_progress(progress,sizeof_file(current_bag.size),".")
    except rosbag.ROSBagUnindexedException:
        print colors.FAIL + "%s unindexed. Run rosbag reindex." % (input) + colors.ENDC
        output.close()
        sys.exit(2)
    try:
        for topic, msg, time in current_bag.read_messages(topics=topic_list):
            output.write(topic, msg, time)
    finally:
        current_bag.close()


def merge_bags(bags_to_merge, output, topic_list, folder):
    """ Merges all bags into one """
    global number_of_bags
    number_of_bags = len(bags_to_merge)
    print colors.OKBLUE + "%d bags to merge in %s" % (number_of_bags, folder) + colors.ENDC
    counter = 1
    for bag in bags_to_merge:
        append_bag(folder + bag, output, topic_list)
        counter += 1


def get_arguments(args):
    """ Parse arguments """
    folder = "."
    name = ""
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
            print colors.WARNING + "Subscribing to all topics..." + colors.ENDC
            add_all_topics(topic_list)
            name += "_all"
        elif opt == "-i":
            print colors.WARNING + "Subscribing to IMU topics..." + colors.ENDC
            add_imu_topics(topic_list)
            name += "_imu"
        elif opt == "-t":
            print colors.WARNING + "Subscribing to temperature topics..." + colors.ENDC
            add_temperature_topics(topic_list)
            name += "_temp"
        elif opt == "-c":
            print colors.WARNING + "Subscribing to camera topics..." + colors.ENDC
            add_camera_topics(topic_list)
            name += "_cam"
        elif opt == "-d":
            print colors.WARNING + "Subscribing to depth topics..." + colors.ENDC
            add_depth_topics(topic_list)
            name += "_depth"
        elif opt == "-b":
            print colors.WARNING + "Subscribing to battery voltages topics..." + colors.ENDC
            add_battery_topics(topic_list)
            name += "_batt"
        elif opt == "-e":
            print colors.WARNING + "Subscribing to electrical interface topics..." + colors.ENDC
            add_electrical_interface_topics(topic_list)
            name += "_elec"
        elif opt == "-h":
            print colors.WARNING + "Subscribing to hydrophones topics..." + colors.ENDC
            add_hydrophones_topics(topic_list)
            name += "_hydro"
        elif opt == "-s":
            print colors.WARNING + "Subscribing to state estimation topics..." + colors.ENDC
            add_state_estimation_topics(topic_list)
            name += "_state"
        elif opt in ("-f", "--folder"):
            folder = arg

    folder = path.abspath(folder)
    name = path.basename(path.normpath(folder)) + name + ".bag"
    if not folder.endswith("/"):
        folder = folder + "/"

    return folder, name, topic_list


def main(folder, name, topic_list):
    """ Set up and merge topics from bags in folder """
    start_time = time.time()
    all_bags = get_bags(folder)
    bags_to_merge = sort_bags(all_bags)
    if len(topic_list) == 0:
        print colors.FAIL + "Please specify at least one topic to subscribe to..." + colors.ENDC
        sos()
        sys.exit(1)
    if len(bags_to_merge) >= 1:
        try:
            output_bag = rosbag.Bag(folder + name, "w")
            merge_bags(bags_to_merge, output_bag, topic_list, folder)
        finally:
            end_time = time.time()
            delta_time = "%ds" % (round(end_time - start_time))
            print colors.WARNING + format_progress("Merged",sizeof_file(output_bag.size),".") + colors.ENDC
            print colors.WARNING + format_progress("Time",delta_time,".") + colors.ENDC
            print colors.OKGREEN + "Bag saved to %s%s" % (folder, name) + colors.ENDC
            output_bag.close()
    else:
        print colors.FAIL + "No bags to merge in %s" % (folder) + colors.ENDC
        sys.exit(1)


if __name__ == "__main__":
    folder, name, topic_list = get_arguments(sys.argv[1:])
    main(folder, name, topic_list)
