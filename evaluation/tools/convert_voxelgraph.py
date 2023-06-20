import rosbag
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import argparse
import os

def main():
    parser = argparse.ArgumentParser("convert ethz voxelgraph demo dataset to odometry")
    parser.add_argument("input", help="input rosbag")
    parser.add_argument("output", help="output rosbag")

    args = parser.parse_args()

    input_bag = rosbag.Bag(args.input,'r')
    output_bag = rosbag.Bag(args.output, 'w')

    msg:PoseWithCovarianceStamped
    for topic, msg, t in input_bag.read_messages():
        if(topic=="/penguin/piksi_node/enu_pose_best_fix"):
            odom_msg = Odometry()
            odom_msg.header=msg.header
            odom_msg.pose = msg.pose
            output_bag.write(topic="/penguin/piksi_node/odometry",msg=odom_msg, t=t)

    output_bag.close()


if __name__ == "__main__":
    main()