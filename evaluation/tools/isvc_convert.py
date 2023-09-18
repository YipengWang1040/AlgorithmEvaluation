import rospy 
import rosbag
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
import tf
import tf.transformations
import argparse
import os
import sys
import shutil
from math import sin, cos, pi
import yaml
import numpy as np

def se3_inv(T):
    ret = np.identity(4)
    ret[0:3,0:3]=T[0:3,0:3].T
    ret[0:3,3]=-np.dot(ret[0:3,0:3],T[0:3,3])
    return ret

# returns idx
def find_transform(tf_msg:TFMessage, child_frame):
    for i in range(len(tf_msg.transforms)):
        if(tf_msg.transforms[i].child_frame_id == child_frame):
            return i
    return -1

# return mat2tf(tf2mat(b)*tf*mat(a))
def multiply_transform(a:TransformStamped, b:TransformStamped):
    Ta = tf.transformations.quaternion_matrix([a.transform.rotation.x,a.transform.rotation.y,a.transform.rotation.z,a.transform.rotation.w])
    Ta[0:3,3]=np.array([a.transform.translation.x,a.transform.translation.y,a.transform.translation.z])
    
    Tb = tf.transformations.quaternion_matrix([b.transform.rotation.x,b.transform.rotation.y,b.transform.rotation.z,b.transform.rotation.w])
    Tb[0:3,3]=np.array([b.transform.translation.x,b.transform.translation.y,b.transform.translation.z])

    T = Tb @ Ta
    
    ret = TransformStamped()
    ret.header = b.header
    ret.child_frame_id = a.child_frame_id
    q=tf.transformations.quaternion_from_matrix(T)
    t=T[0:3,3]
    ret.transform.rotation.x = q[0]
    ret.transform.rotation.y = q[1]
    ret.transform.rotation.z = q[2]
    ret.transform.rotation.w = q[3]
    ret.transform.translation.x = t[0]
    ret.transform.translation.y = t[1]
    ret.transform.translation.z = t[2]

    return ret

# pt_tgt = transform_to(src,tgt) @ pt_src
def transform_to(tf_msg:TFMessage, source, target):
    idx = find_transform(tf_msg,source)
    if(idx<0):
        raise RuntimeError("{} is not in tf array".format(src(source)))
    src:TransformStamped = tf_msg.transforms[idx]
    if(src.header.frame_id==target):
        return src
    else:
        return multiply_transform(src, transform_to(tf_msg, src.header.frame_id, target))


def parse_tf(config, time:rospy.Time):
    # parse config file
    tf_msg = TFMessage()
    with open(config, 'r') as f:
        yaml_node = yaml.load(f, Loader=yaml.UnsafeLoader)
        links = int(yaml_node["links"])
        transforms=[None]
        frames=[yaml_node["link0"]["name"]]
        for i in range(1,links):
            transforms.append(yaml_node["link{}".format(i)]["transform"])
            frames.append(yaml_node["link{}".format(i)]["name"])
        
        for i in range(1,links):
            transform = TransformStamped()
            transform.header.stamp = time
            T=np.array(transforms[i]['T'])
            if("to" in transforms[i]):
                transform.header.frame_id = frames[transforms[i]["to"]]
                transform.child_frame_id = frames[i]
            elif("from" in transforms[i]):
                T=se3_inv(T)
                transform.header.frame_id = frames[transforms[i]["from"]]
                transform.child_frame_id = frames[i]
                
            q=tf.transformations.quaternion_from_matrix(T)
            t=T[0:3,3]
            transform.transform.rotation.x=q[0]
            transform.transform.rotation.y=q[1]
            transform.transform.rotation.z=q[2]
            transform.transform.rotation.w=q[3]
            transform.transform.translation.x=t[0]
            transform.transform.translation.y=t[1]
            transform.transform.translation.z=t[2]
            tf_msg.transforms.append(transform)

    return tf_msg


# https://en.wikipedia.org/wiki/Geographic_coordinate_system#Length_of_a_degree
def relative_position(lat1, lon1, lat2, lon2):
    lat_mid = (lat1 + lat2) / 2
    m_per_deg_lat = 111132.92 - 559.822 * cos( 2.0 * lat_mid ) + 1.175 * cos( 4.0 * lat_mid) - 0.0023 * cos( 6.0 * lat_mid)
    m_per_deg_lon = 111412.84 * cos ( lat_mid ) -93.5 * cos (3.0 * lat_mid) + 0.118 * cos(5 * lat_mid)

    return (lat2 - lat1) * m_per_deg_lat, (lon2 - lon1) * m_per_deg_lon

def navsetfix2path(msgs, ignore=False)->Path:
    path = Path()
    start_pos = None
    ignored = 0
    for i in range(len(msgs)):
        msg:NavSatFix = msgs[i][0]
        if (start_pos == None):
            start_pos = (msg.latitude, msg.longitude, msg.altitude)
        cur_pose = PoseStamped()
        cur_pose.header.stamp=msg.header.stamp
        cur_pose.header.frame_id="rtk_path_frame"
        delta_xy = relative_position(start_pos[0], start_pos[1], msg.latitude, msg.longitude)
        cur_pose.pose.position.x = delta_xy[0]
        cur_pose.pose.position.y = delta_xy[1]
        cur_pose.pose.position.z = msg.altitude - start_pos[2]
        if(ignore):
            if(msg.status.status!=2):
                ignored += 1
                continue

        path.header=cur_pose.header
        path.poses.append(cur_pose)

    return path, ignored

def main():
    parser = argparse.ArgumentParser("Export navsatfix as odometry. The input is a folder of sequences, where each sequence is in a folder.")
    parser.add_argument("directory", help="directory to the folders of rosbags")
    parser.add_argument("output_directory", help="directory to the output")
    args = parser.parse_args()
    with os.scandir(args.directory) as dirs:
        for dir in dirs:
            if(os.path.isdir(dir)):
                bags = []
                odom_bag_path = None
                with os.scandir(dir) as files:
                    for entry in files:
                        if(entry.name.endswith(".bag")):
                            if(entry.name.startswith("base")):
                                bags.append(entry.name)
                            elif(entry.name.startswith("odom")):
                                odom_bag_path=entry.name
                            
                bags.sort()

                # read all messages of navsatfix
                msgs = []
                first_t = None
                for bag_path in bags:
                    bag=rosbag.Bag(dir.path+'/'+bag_path,'r')
                    for topic, msg, t in bag.read_messages(topics="/piksi/navsatfix_best_fix"):
                        msgs.append((msg,t))
                        if(first_t is None):
                            first_t = t

                odom_bag = rosbag.Bag(dir.path+'/'+odom_bag_path, 'r')
                wheel_odom_msgs=[]
                for topic, msg, t in odom_bag.read_messages(topics="/jackal_velocity_controller/odom"):
                    wheel_odom_msgs.append((msg,t))

                print("Found {} messages in given rosbags. Start converting...".format(len(msgs)))
                
                # construct path
                path, ignored_msgs = navsetfix2path(msgs, True)
                print("Finished Parsing. {} messages ignored due to non-fix rtk status".format(ignored_msgs))

                if(os.path.exists(args.output_directory + '/' + dir.name)):
                    shutil.rmtree(args.output_directory + '/' + dir.name)
                os.mkdir(args.output_directory + '/' + dir.name )
                
                output_bag = rosbag.Bag(args.output_directory + '/' + dir.name + "/"+odom_bag_path,'w')
                p:PoseStamped
                for i in range(len(msgs)):
                    p = path.poses[i]
                    odom = Odometry()
                    odom.pose.pose = p.pose
                    odom.header = p.header
                    output_bag.write("/gps/fix/odometry", odom, t=msgs[i][1])

                for msg in wheel_odom_msgs:
                    output_bag.write("/jackal_velocity_controller/odom",msg[0],msg[1])
                output_bag.close()


                # static tf
                tf_msg = parse_tf(os.path.dirname(os.path.abspath(sys.argv[0]))+'/links.yaml', first_t)
                static_tf_bag = rosbag.Bag(args.output_directory + '/' + dir.name+'/static_tf.bag','w')
                static_tf_bag.write("/tf_static", tf_msg, first_t)
                static_tf_bag.close()


if __name__ == "__main__":
    main()
