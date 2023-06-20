import rosbag
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import argparse
import os
import numpy as np
import tf.transformations

def transform_pose(T, pose:Pose)->Pose:
    T2=tf.transformations.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    T2[0:3,3]=[pose.position.x, pose.position.y, pose.position.z]
    
    result=T@T2
    #result[0:3,0:3]=result[0:3,0:3]@T[0:3,0:3].T
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

def inverse_pose(pose:Pose):
    T=tf.transformations.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    T[0:3,3]=[pose.position.x, pose.position.y, pose.position.z]
    return tf.transformations.inverse_matrix(T)

def main():
    parser = argparse.ArgumentParser("convert ncd ground truth to bag of odometry")
    parser.add_argument("input", help="input rosbag")
    parser.add_argument("output", help="output rosbag")

    args = parser.parse_args()

    output_bag = rosbag.Bag(args.output, 'w')

    T = None
    with open(args.input,'r') as f:
        for line in f:
            if(line.startswith('#')):
                continue
            data=line.split(',')
            odom_msg = Odometry()
            odom_msg.header.frame_id = "world"
            odom_msg.header.stamp = rospy.Time(int(data[0]), int(data[1]))
            odom_msg.pose.pose.position.x = float(data[2])
            odom_msg.pose.pose.position.y = float(data[3])
            odom_msg.pose.pose.position.z = float(data[4])
            odom_msg.pose.pose.orientation.x = float(data[5])
            odom_msg.pose.pose.orientation.y = float(data[6])
            odom_msg.pose.pose.orientation.z = float(data[7])
            odom_msg.pose.pose.orientation.w = float(data[8])
            if(T is None):
                T= tf.transformations.euler_matrix(0,0,-135/180*np.pi)@inverse_pose(odom_msg.pose.pose)
            odom_msg.pose.pose = transform_pose(T,odom_msg.pose.pose)
            output_bag.write(topic="/ncd/gt/odometry",msg=odom_msg, t=odom_msg.header.stamp)

    output_bag.close()


if __name__ == "__main__":
    main()