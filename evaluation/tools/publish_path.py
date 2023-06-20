import rospy 
import rosbag
import tf
import tf.transformations
import tf2_ros
import argparse
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
import os
from math import sin, cos, pi

def bag2path(path, frame_id):
    print("reading bag from \"{}\"...".format(path))
    if(os.path.isdir(path)):
        bags=[f for f in os.listdir(path) if f.endswith('.bag')]
        if(len(bags)!=1):
            return None
        bag_file=bags[0]
        bag=rosbag.Bag(path+'/'+bag_file,'r')
    else:
        bag = rosbag.Bag(path, 'r')
    odom_topic=""
    max_messages=0
    max_topic=""
    for topic, topic_tuple in bag.get_type_and_topic_info()[1].items():
        if(topic_tuple.msg_type=='nav_msgs/Odometry'):
            if(bag.get_message_count(topic)>max_messages):
                max_messages=bag.get_message_count(topic)
                max_topic=topic
    odom_topic=max_topic

    path=Path()
    for topic, msg, t in bag.read_messages(topics=odom_topic):
        cur_pose = PoseStamped()
        cur_pose.header.stamp=rospy.Time()
        cur_pose.header.frame_id=frame_id
        cur_pose.pose=msg.pose.pose

        path.header=cur_pose.header
        path.poses.append(cur_pose)

    print("Found {} poses in the bag. Path message built.".format(len(path.poses)))
    
    return path

def main():
    parser = argparse.ArgumentParser("publish path given a rosbag contains lots of odometries")
    parser.add_argument("bag", help="file name")
    parser.add_argument("frame_id", help="frame id for resulted path")
    args = parser.parse_args()

    path = bag2path(args.bag, args.frame_id)
    
    rospy.init_node('path_publisher')
    r = rospy.Rate(10)
    pub = rospy.Publisher("/path_generated", Path, queue_size=10)
    while(not rospy.is_shutdown()):
        pub.publish(path)
        r.sleep()


if __name__ == "__main__":
    main()