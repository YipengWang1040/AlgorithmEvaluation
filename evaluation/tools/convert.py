import rospy 
import rosbag
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import tf
import tf.transformations
import argparse
import os
import shutil
from math import sin, cos, pi


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
        msg:NavSatFix = msgs[i]
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

def export_ground_truth(filename, path:Path):
    bag = rosbag.Bag(filename, 'w')
    p:PoseStamped
    for p in path.poses:
        odom = Odometry()
        odom.pose.pose = p.pose
        odom.header = p.header
        bag.write("/gt_odometry",odom)
    bag.close()

def main():
    parser = argparse.ArgumentParser("Publish navsatfix as path. The input is a folder of sequences, where each sequence is in a folder.")
    parser.add_argument("directory", help="directory to the folders of rosbags")
    parser.add_argument("output_directory", help="directory to the output")
    parser.add_argument("--ignore", help="ignoring non-rtk-fix segments", action="store_true")
    args = parser.parse_args()
    with os.scandir(args.directory) as dirs:
        for dir in dirs:
            if(os.path.isdir(dir)):
                bags = []
                with os.scandir(dir) as files:
                    for entry in files:
                        if(entry.name.endswith(".bag")):
                            bags.append(entry.name)
                            
                bags.sort()

                # read all messages of navsatfix
                msgs = []
                for bag_path in bags:
                    bag=rosbag.Bag(dir.path+'/'+bag_path,'r')
                    for topic, msg, t in bag.read_messages(topics="/piksi/navsatfix_best_fix"):
                        msgs.append(msg)

                print("Found {} messages in given rosbags. Start converting...".format(len(msgs)))
                
                # construct path
                path, ignored_msgs = navsetfix2path(msgs, args.ignore)
                print("Finished Parsing. {} messages ignored due to non-fix rtk status".format(ignored_msgs))

                if(os.path.exists(args.output_directory + '/' + dir.name)):
                    shutil.rmtree(args.output_directory + '/' + dir.name)
                os.mkdir(args.output_directory + '/' + dir.name )
                export_ground_truth(args.output_directory + '/' + dir.name + "/ground_truth_exported.bag", path)


if __name__ == "__main__":
    main()
