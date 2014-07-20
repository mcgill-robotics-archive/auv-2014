#!/usr/bin/env python

# IMPORTS
import rosbag
import sys
import getopt
import time
from os import path, popen, walk

# PARSING CONDITIONS
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
    print "         -f | -folder : folder containing bags"
    print "         -F | --from : bag number to start merging from"
    print "         -T | --to : bag number to end merging at"
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
    # CHECK PARSING CONDITIONS
    starts_correctly = filename.startswith(prefix + delimiter)
    ends_correctly = filename.endswith(extension)
    not_incorrect = ("orig" not in filename) and ("active" not in filename)

    return starts_correctly and ends_correctly and not_incorrect


def get_bag_count(filename):
    """ Parses filename for correct order """
    return int(filename.split(delimiter)[1].split(".")[0])


def sort_bags(all_bags):
    """ Sorts bags in the correct order """
    bag_numbers = []
    bags_to_merge = []

    # PARSE FILENAMES TO SORT PROPERLY
    for filename in all_bags:
        bag_numbers.append(get_bag_count(filename))
    bag_numbers.sort()

    # FIX FILENAMES
    for i in bag_numbers:
        bags_to_merge.append("%s%s%d%s" % (prefix, delimiter, i, extension))

    return bags_to_merge


def get_bags(folder, t0, t1):
    """ Gets list of bags in range from folder """
    all_bags = []
    all_files = []

    # GET FILES IN DIRECTORY
    for (dirpath, dirnames, filenames) in walk(folder):
        all_files.extend(filenames)
        break

    # GET BAGS FROM FILE LIST
    for filename in all_files:
        if bag_we_are_looking_for(filename):
            all_bags.append(filename)

    # SORT LIST
    bags_to_merge = sort_bags(all_bags)

   # CHECK IF BAG LIST IS IN RANGE
    if t0 >= len(bags_to_merge) or t1 > len(bags_to_merge):
        print colors.FAIL + "Bags out of range" + colors.ENDC
        sos()
        sys.exit(1)

    # GET FILES IN RANGE
    bags_to_merge_in_range = []
    for index in range(len(bags_to_merge)):
        bag_number = get_bag_count(bags_to_merge[index])
        if bag_number >= t0 and (t1 == -1 or bag_number <= t1):
             bags_to_merge_in_range.append(bags_to_merge[index])

    return bags_to_merge_in_range


def append_bag(input, output, topic_list, counter):
    """ Appends bag data """
    # OPEN BAG
    try:
        current_bag = rosbag.Bag(input, "r")
        progress = "Merging %s (%d of %d)" % (path.basename(path.normpath(input)), counter, number_of_bags)
        print format_progress(progress,sizeof_file(current_bag.size),".")
    except rosbag.ROSBagUnindexedException:
        print colors.FAIL + "%s unindexed. Run rosbag reindex." % (input) + colors.ENDC
        output.close()
        sys.exit(2)

    # APPEND MESSAGES
    try:
        for topic, msg, time in current_bag.read_messages(topics=topic_list):
            output.write(topic, msg, time)
    finally:
        current_bag.close()


def merge_bags(bags_to_merge, output, topic_list, folder):
    """ Merges all bags into one """
    global number_of_bags
    number_of_bags = len(bags_to_merge)
    print colors.HEADER + "%d bags to merge in %s" % (number_of_bags, folder) + colors.ENDC

    # MERGE
    counter = 1
    for bag in bags_to_merge:
        append_bag(folder + bag, output, topic_list, counter)
        counter += 1


def get_arguments(args):
    """ Parse arguments """
    folder = "."
    name = ""
    topic_list = []
    t0, t1 = 0, -1

    # CHECK IF ARGUMENTS WERE SPECIFIED
    if len(args) == 0:
        sos()
        sys.exit(1)

    # GET ARGUMENTS
    try:
        opts, args = getopt.getopt(args,"aitcdbehsf:F:T:",["help=","folder=","from=","to="])
    except getopt.GetoptError:
        sos()
        sys.exit(2)

    # PARSE ARGUMENTS
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
        elif opt in ("-F", "--from"):
            try:
                t0 = int(arg)
            except ValueError:
                print colors.FAIL + "Invalid arguments. Expected integer." + colors.ENDC
                sos()
                sys.exit(1)
        elif opt in ("-T", "--to"):
            try:
                t1 = int(arg)
            except ValueError:
                print colors.FAIL + "Invalid arguments. Expected integer." + colors.ENDC
                sos()
                sys.exit(1)

    # CHECK IF RANGE OF BAGS IS REASONABLE
    if t1 != -1 and t1 < t0:
        print colors.FAIL + "Invalid bag range" + colors.ENDC
        sos()
        sys.exit(1)

    # CHECK IF TOPICS WERE SPECIFIED PROPERLY
    if len(topic_list) == 0:
        print colors.FAIL + "Please specify at least one topic to subscribe to..." + colors.ENDC
        sos()
        sys.exit(1)

    # FIX FOLDER AND FILE NAMES
    folder = path.abspath(folder)
    name = path.basename(path.normpath(folder)) + name + ".bag"
    if not folder.endswith("/"):
        folder = folder + "/"

    return folder, name, topic_list, t0, t1


def main():
    """ Set up and merge topics from bags in folder """
    start_time = time.time()

    # GET ARGUMENTS
    folder, name, topic_list, t0, t1 = get_arguments(sys.argv[1:])

    # GET BAGS FROM t0 TO t1
    bags_to_merge = get_bags(folder, t0, t1)

    # CHECK IF ENOUGH BAGS TO MERGE EXIST
    if len(bags_to_merge) >= 1:
        # CREATE AND MERGE BAGS
        try:
            output = rosbag.Bag(folder + name, "w")
            merge_bags(bags_to_merge, output, topic_list, folder)

        # PRINT FOOTER AND CLOSE BAG
        finally:
            end_time = time.time()
            delta_time = "%ds" % (round(end_time - start_time))
            print colors.OKBLUE + format_progress("Merged",sizeof_file(output.size),".") + colors.ENDC
            print colors.OKBLUE + format_progress("Time",delta_time,".") + colors.ENDC
            print colors.OKGREEN + "Bag saved to %s%s" % (folder, name) + colors.ENDC
            output.close()
    else:
        print colors.FAIL + "No bags in range to merge in %s" % (folder) + colors.ENDC
        sys.exit(1)


# START HERE
if __name__ == "__main__":
    main()
