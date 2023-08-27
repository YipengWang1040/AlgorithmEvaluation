import rosbag
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import argparse
import os

def find_closest_index(L, t, beginning=0):
    difference = abs(L[beginning].header.stamp.to_sec() - t)
    best = beginning
    end = len(L)
    while beginning < end:
        middle = int((end+beginning)/2)
        if abs(L[middle].header.stamp.to_sec() - t) < difference:
            difference = abs(L[middle].header.stamp.to_sec() - t)
            best = middle
        if t == L[middle].header.stamp.to_sec():
            return middle
        elif L[middle].header.stamp.to_sec() > t:
            end = middle
        else:
            beginning = middle + 1
    return best

def main():
    parser = argparse.ArgumentParser("trim bacchus to remain only used topics")
    parser.add_argument("input_dir", help="input directory")
    parser.add_argument("output_dir", help="output directory")
    parser.add_argument("--topics", nargs='+', help="topics to reserve", required=True)

    args = parser.parse_args()

    bags = os.listdir(args.input_dir)
    for bag in bags:
        if(bag.endswith(".bag")):
            print("trimming {}".format(bag))
            trim_bag(args.input_dir + '/' + bag, args.output_dir+'/'+bag, args.topics)


def trim_bag(input_file, output_file, topics):
    input_bag = rosbag.Bag(input_file, 'r')
    output_bag = rosbag.Bag(output_file, 'w')
    
    for topic, msg, t in input_bag.read_messages(topics = topics):
        if(hasattr(msg, "header")):
            msg.header.frame_id = "velodyne"
        output_bag.write(topic, msg, t)

    output_bag.close()

if __name__ == "__main__":
    main()