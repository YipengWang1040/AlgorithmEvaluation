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
    parser = argparse.ArgumentParser("trim nclt groundtruth to a 1:1 amount to lidar")
    parser.add_argument("input_dir", help="input directory")
    parser.add_argument("output_dir", help="output directory")

    args = parser.parse_args()

    bags = os.listdir(args.input_dir)
    for bag in bags:
        if(bag.endswith(".bag")):
            print("trimming {}".format(bag))
            trim_bag(args.input_dir + '/' + bag, args.output_dir+'/'+bag)


def trim_bag(input_file, output_file):
    input_bag = rosbag.Bag(input_file, 'r')
    output_bag = rosbag.Bag(output_file, 'w')
    
    gt_buffer = []
    velodyne_stamp_buffer = []
    for topic, msg, t in input_bag.read_messages():
        if(topic=="/rslidar_points"):
            velodyne_stamp_buffer.append(msg.header.stamp)
        if(topic=="/track_odometry"):
            gt_buffer.append(msg)
    
    gt_output = []
    skipped_velodyne_indices = []
    beginning = 0
    for i in range(len(velodyne_stamp_buffer)):
        stamp = velodyne_stamp_buffer[i]
        beginning = find_closest_index(gt_buffer, stamp.to_sec(), beginning)
        if(abs(gt_buffer[beginning].header.stamp.to_nsec() - stamp.to_nsec())>30000000):
            skipped_velodyne_indices.append(i)
            continue
        gt_output.append(gt_buffer[beginning])
    
    for msg in gt_output:
        output_bag.write(topic="/track_odometry",msg=msg, t=msg.header.stamp)

    output_bag.close()

    with open(output_file+".skipped.txt",'w') as f:
        for i in skipped_velodyne_indices:
            f.write("{}\n".format(i))

    # debug
    diffs=[]
    with open("debug_output.txt",'w') as f:
        for i in range(len(gt_output)):
            diffs.append(abs(velodyne_stamp_buffer[i].to_nsec() - gt_output[i].header.stamp.to_nsec()))
            f.write("{}:{},    {}\n".format(velodyne_stamp_buffer[i].to_nsec(), gt_output[i].header.stamp.to_nsec(), abs(velodyne_stamp_buffer[i].to_nsec() - gt_output[i].header.stamp.to_nsec())))
    diffs.sort()
    with open("debug_output2.txt",'w') as f:
        for i in range(len(diffs)):
            f.write("{}\n".format(diffs[i]))


if __name__ == "__main__":
    main()