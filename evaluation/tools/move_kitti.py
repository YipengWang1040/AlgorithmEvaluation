import rosbag
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import argparse
import os

def main():
    parser = argparse.ArgumentParser("trim nclt groundtruth to a 1:1 amount to lidar")
    parser.add_argument("input_dir", help="input directory")
    parser.add_argument("output_dir", help="output directory")

    args = parser.parse_args()

    bags = os.listdir(args.input_dir)
    for bag in bags:
        if(bag.endswith(".bag")):
            print("moving {}...".format(bag))
            move_odometry(args.input_dir + '/' + bag, args.output_dir+'/'+bag)


def move_odometry(input_file, output_file):
    input_bag = rosbag.Bag(input_file, 'r')
    output_bag = rosbag.Bag(output_file, 'w')
    

    for topic, msg, t in input_bag.read_messages(topics="/kitti/odometry"):
        output_bag.write("/kitti/odometry", msg, t)
    
    output_bag.close()


if __name__ == "__main__":
    main()