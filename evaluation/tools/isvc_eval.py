

import os
import argparse
import shutil
import string
import json
import rosbag
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import yaml
import tf.transformations
import numpy as np

from ate import ate


def transform_pose(T, pose:Pose)->Pose:
    T2=tf.transformations.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    T2[0:3,3]=[pose.position.x, pose.position.y, pose.position.z]
    
    result=T2@T
    result[0:3,0:3]=result[0:3,0:3]@T[0:3,0:3].T
    result_p=result[0:3,3]
    result_q=tf.transformations.quaternion_from_matrix(result)
    ret=Pose()
    ret.position.x=result_p[0]
    ret.position.y=result_p[1]
    ret.position.z=result_p[2]
    ret.orientation.x=result_q[0]
    ret.orientation.y=result_q[1]
    ret.orientation.z=result_q[2]
    ret.orientation.w=result_q[3]
    return ret

def load_bag(filename):
    print("reading input bag from \"{}\"...".format(filename))
    bag = rosbag.Bag(filename, 'r')
    odom_topic=""
    max_messages=0
    max_topic=""
    for topic, topic_tuple in bag.get_type_and_topic_info()[1].items():
        if(topic_tuple.msg_type=='nav_msgs/Odometry'):
            if(bag.get_message_count(topic)>max_messages):
                max_messages=bag.get_message_count(topic)
                max_topic=topic
    odom_topic=max_topic
    print("using topic \"{}\"".format(odom_topic))

    rotation=[0,0,0,1]
    translation=[-0.21,0,0.1553]
    T=tf.transformations.quaternion_matrix(rotation)
    T[0:3,3]=translation

    path=Path()
    for topic, msg, t in bag.read_messages(topics=odom_topic):
        cur_pose = PoseStamped()
        cur_pose.header.stamp=msg.header.stamp
        cur_pose.pose=msg.pose.pose
        cur_pose.pose=transform_pose(T,cur_pose.pose)

        path.header=cur_pose.header
        path.poses.append(cur_pose)

    return path

# timestamp(ns) x y z qx qy qz qw
def load_csv(filename):
    print("reading input csv from \"{}\"...".format(filename))

    rotation=[0,0,0,1]
    translation=[-0.21,0,0.1553]
    T=tf.transformations.quaternion_matrix(rotation)
    T[0:3,3]=translation

    path=Path()
    with open(filename) as f:
        lines = f.readlines()
        for line in lines:
            vals = line.split()
            cur_pose = PoseStamped()
            cur_pose.header.stamp = rospy.Time(int(vals[0])//1000000000, int(vals[0])%1000000000)
            cur_pose.pose.position.x = float(vals[1])
            cur_pose.pose.position.y = float(vals[2])
            cur_pose.pose.position.z = float(vals[3])
            cur_pose.pose.orientation.x = float(vals[4])
            cur_pose.pose.orientation.y = float(vals[5])
            cur_pose.pose.orientation.z = float(vals[6])
            cur_pose.pose.orientation.w = float(vals[7])
            cur_pose.pose = transform_pose(T, cur_pose.pose)

            path.header=cur_pose.header
            path.poses.append(cur_pose)
        
    return path

def load_gt(filename):
    print("reading input bag from \"{}\"...".format(filename))
    bag = rosbag.Bag(filename, 'r')

    path=Path()
    for topic, msg, t in bag.read_messages(topics="/gps/fix/odometry"):
        cur_pose = PoseStamped()
        cur_pose.header.stamp=msg.header.stamp
        cur_pose.pose=msg.pose.pose

        path.header=cur_pose.header
        path.poses.append(cur_pose)

    return path

def main():
    parser = argparse.ArgumentParser("Evaluate results of lidar odometry")
    parser.add_argument("trajectory", help="trajectory output. can either be csv or rosbag contains a topic of \"nav_msgs/Odometry\".")
    parser.add_argument("ground_truth", help="ground truth bag \"odom_xxxx.bag\". ")

    args = parser.parse_args()

    if(args.trajectory.endswith("bag")):
        traj=load_bag(args.trajectory)
    elif(args.trajectory.endswith("csv")):
        traj=load_csv(args.trajectory)
    else:
        print("Please input a csv file or a rosbag")
        return
    
    gt=load_gt(args.ground_truth)


    rot, trans, trans_error = ate(traj,gt)
    print(np.mean(trans_error))

if __name__ == "__main__":
    main()