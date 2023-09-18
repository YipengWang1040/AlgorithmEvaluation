
# all datasets
# no space should be in any of the topics
DATASETS={
    # "agops_move":{
    #     "location":              "/media/arcs/ARCS-Drive/jackal_dataset/agops/move/trimmed",
    #     "ground_truth":          "/media/arcs/ARCS-Drive/jackal_dataset/agops/move/ground_truth",
    #     "lidar_topic":           "/velodyne_points",
    #     "imu_topic":             "/microstrain/imu/data",
    #     "sequences": [
    #         "move1",
    #         "move2",
    #         "move3",
    #         "move4",
    #         "move5",
    #         "move6",
    #         "move7",
    #         "move8",
    #         "move9",
    #         "move10",
    #     ]
    # },
    # "bacchus":{
    #     "location":              "/media/arcs/ARCS-Drive/LiDAR Datasets/BACCHUS Long Term Agriculture Dataset/trimmed",
    #     "ground_truth":          "/home/arcs/dataset/BACCHUS/ground_truth/",
    #     "lidar_topic":           "/os_cloud_node/points",
    #     "imu_topic":             "/imu/data",
    #     "sequences": [
    #         "2022-03-23-12-27-15",
    #     #    "2022-04-20-09-37-20",    # corrupted
    #         "2022-05-06",
    #         "2022-06-01-15-16-04",
    #     ]
    # },

    "agops_isvc":{
        "location":              "/media/arcs/ARCS-Drive/jackal_dataset/agops/full_trimmed",
        "ground_truth":          "/home/arcs/dataset/agops/ground_truth/isvc",
        "lidar_topic":           "/velodyne_points",
        "imu_topic":             "/microstrain/imu/data",
        "extrinsic": [0, 0, 0, 1,   -0.21, 0, 0.1553], # qx, qy, qz, qw, x, y, z; ground truth frame in lidar frame
        "sequences": [
            "full_6",
            "full_9",
            "full_10",
            "full_11",
            "full_12",
            "full_13",
            "full_14",
        ]
    },
    # "hilti":{
    #     "location":              "/home/arcs/dataset/hilti",
    #     "ground_truth":          "/home/arcs/dataset/hilti/ground_truth",
    #     "lidar_topic":           "/rslidar_points",
    #     "imu_topic":             "/imu/data",
    #     "sequences": [
    #         "site2_robot_1",
    #     ],
    #     "play_topics": [
            
    #     ]
    # },

    # "hilti":{
    #     "location":              "/home/arcs/dataset/hilti",
    #     "ground_truth":          "/home/arcs/dataset/hilti/ground_truth",
    #     "lidar_topic":           "/rslidar_points",
    #     "imu_topic":             "/imu/data",
    #     "sequences": [
    #         "site2_robot_1",
    #     ],
    # },

    "agops":{
        "location":              "/media/arcs/ARCS-Drive/jackal_dataset/agops/",
        "ground_truth":          "/home/arcs/dataset/agops/ground_truth",
        "lidar_topic":           "/velodyne_points",
        "imu_topic":             "/microstrain/imu/data",
        "sequences": [
            # "row_short_easy1",
            # "row_short_easy2",
            "row_short_hard1",
            "row_short_hard2",
            # "row_long_hard1",
            # "row_long_hard2",
            # "row_long_lawn_mower",
            "uniform_short_easy",
            # "uniform_short_hard",
            # "uniform_long_lawn_mower",
            # "uniform_long_cross_move",
        ]
    },
    "lio_sam":{
        "location":              "/media/arcs/ARCS-Drive/LiDAR Datasets/lio_sam",
        "ground_truth":          "/media/arcs/ARCS-Drive/LiDAR Datasets/lio_sam/ground_truth",
        "lidar_topic":           "/points_raw",
        "imu_topic":             "/imu_correct",
        "sequences": [
            "walking",
            "campus_small",
        ]
    },
    # "agops_rtk2":{
    #     "location":              "/media/arcs/ARCS-Drive/jackal_dataset/agops/RTK data4",
    #     "ground_truth":          "/home/arcs/dataset/agops/ground_truth/rtk2",
    #     "lidar_topic":           "/velodyne_points",
    #     "imu_topic":             "/microstrain/imu/data",
    #     "sequences": [
    #         "seq4",
    #         "seq6",
    #         "seq7",
    #         "seq8",
    #         "seq9",
    #         "seq10",
    #         "seq11",
    #     ]
    # },
    # "agops_rtk":{
    #     "location":              "/home/arcs/dataset/agops/rtk",
    #     "ground_truth":          "/home/arcs/dataset/agops/ground_truth/rtk",
    #     "lidar_topic":           "/velodyne_points",
    #     "imu_topic":             "/microstrain/imu/data",
    #     "sequences": [
    #         "seq6",
    #         "seq7",
    #         "seq8",
    #         "seq9",
    #     ]
    # },

    # "kitti":{
    #     "location":              "/media/arcs/trash/dataset/KITTI_Odometry",
    #     "ground_truth":          "/home/arcs/dataset/kitti/ground_truth",
    #     "lidar_topic":           "/kitti/velodyne_points",
    #     "imu_topic":             "/imu",
    #     "sequences": [
    #         "00",
    #         "02",
    #     ]
    # },
    # "nclt":{
    #     "location":              "/media/arcs/ARCS-Drive/LiDAR Datasets/nclt/rosbags",
    #     "ground_truth":          "/home/arcs/dataset/nclt/trimmed_ground_truth",
    #     "lidar_topic":           "/nclt/velodyne_points",
    #     "imu_topic":             "/nclt/imu",
    #     "sequences": [
    #         "2012-01-08",
    #         # "2012-01-15",
    #         # "2012-01-22",
    #         # "2012-02-02",
    #         # "2012-02-04",
    #         # "2012-02-05",
    #         # "2012-02-12",
    #         # "2012-02-18",
    #         # "2012-02-19",
    #         "2012-03-17",
    #         # "2012-03-25",    # <- this one is broken
    #         # "2012-03-31",
    #         # "2012-04-29",
    #         # "2012-05-11",
    #         # "2012-05-26",
    #         # "2012-06-15",
    #         # "2012-08-04",
    #     ],
    # },

    # "ncd":{
    #     "location":              "/home/arcs/dataset/ncd",
    #     "ground_truth":          "/home/arcs/dataset/ncd/ground_truth",
    #     "lidar_topic":           "/os1_cloud_node/points",
    #     "imu_topic":             "/os1_cloud_node/imu",
    #     "sequences": [
    #         "01_short_experiment",
    #         # "02_long_experiment"
    #     ],
    # },

    # "voxgraph":{
    #     "location":              "/home/arcs/dataset/Voxgraph/demo",
    #     "ground_truth":          "/home/arcs/dataset/Voxgraph/ground_truth",
    #     "lidar_topic":           "/penguin/os1_cloud_node/points",
    #     "imu_topic":             "/penguin/os1_cloud_node/imu",
    #     "sequences": [
    #         "2020_voxgraph_arche",
    #     ],
    # },

    # "mulran":{
    #     "location":              "/home/arcs/dataset/mulran",
    #     "ground_truth":          "/home/arcs/dataset/mulran/ground_truth",
    #     "lidar_topic":           "/mulran/os1_points",
    #     "imu_topic":             "/mulran/imu",
    #     "sequences": [
    #         "DCC01",
    #         "KAIST01",
    #         "Riverside01",
    #     ],
    # },











    # # test dataset
    # "agops_rtk":{
    #     "location":              "/media/arcs/trash/dataset/agops/rtk",
    #     "lidar_topic":           "/velodyne_points",
    #     "imu_topic":             "/microstrain/imu/data",
    #     "sequences": [
    #         "seq1",
    #         "seq2",
    #     ]
    # },
}

# all algotirhms
ALGORITHMS={
    # "color_loam_new":{
    #     "session_name": "color_loam_new_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/color_loam_new.yaml",
    #         "-d",
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/microstrain/imu/data"
    #     },
    #     "output_topics": [
    #         "/odometry",
    #         "/execution_time"
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     #    "-u 10"
    #     ]
    # },
    # "lio-sam":{
    #     "session_name": "lio-sam_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/lio_sam.yaml",
    #         "-d",
    #     ],
    #     # optional
    #     # "clean_command":[
    #     #     "bash",
    #     #     "clean_scripts/lego_loam.sh"
    #     # ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/microstrain/imu/data"
    #     },
    #     "output_topics": [
    #         "/robot/dlo/odom_node/odom",
    #         "/execution_time"
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     #    "-u 10"
    #     ]
    # },
    # "dlo":{
    #     "session_name": "dlo_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/dlo.yaml",
    #         "-d",
    #     ],
    #     # optional
    #     # "clean_command":[
    #     #     "bash",
    #     #     "clean_scripts/lego_loam.sh"
    #     # ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/microstrain/imu/data"
    #     },
    #     "output_topics": [
    #         "/robot/dlo/odom_node/odom",
    #         "/execution_time"
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     #    "-u 10"
    #     ]
    # },

    # "lego_loam":{
    #     "session_name": "lego_loam_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/lego_loam.yaml",
    #         "-d",
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/microstrain/imu/data"
    #     },
    #     "output_topics": [
    #         "/integrated_to_init",
    #         "/execution_time"
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     #    "-u 10"
    #     ]
    # },
    # "ct_icp":{
    #     "session_name": "ct_icp_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/ct_icp.yaml",
    #         "-d",
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/ct_icp/pointcloud",
    #         "imu_topic": "/microstrain/imu/data"
    #     },
    #     "output_topics": [
    #         "/ct_icp/pose/odom",
    #         "/execution_time"
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #         "-r 0.33",
    #     #    "-u 10"
    #     ]
    # },
    # "hdl_graph_slam":{
    #     "session_name": "hdl_graph_slam_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/hdl_graph_slam.yaml",
    #         "-d",
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/imu/data"
    #     },
    #     "output_topics": [
    #         "/odom",
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     ]
    # },
    # "kiss_icp":{
    #     "session_name": "kiss_icp_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/kiss_icp.yaml",
    #         "-d",
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/imu/data"
    #     },
    #     "output_topics": [
    #         "/odometry_node/odometry",
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #         "-r 0.5"
    #     ]
    # },
    "kiss_icp2":{
        "session_name": "kiss_icp2_session",
        "launch_command": [
            "tmuxp",
            "load",
            "launch_scripts/kiss_icp2.yaml",
            "-d",
        ],
        "input_topics": {
            "lidar_topic": "/velodyne_points",
            "imu_topic": "/imu/data"
        },
        "output_topics": [
            "/odometry_node/odometry",
        ],
        "rosbag_args": [
            "--clock",
            "-r 0.333"
        ]
    },
    # "aloam":{
    #     "session_name": "aloam_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/aloam.yaml",
    #         "-d",
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/imu/data"
    #     },
    #     "output_topics": [
    #         "/aft_mapped_to_init_high_frec",
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     ]
    # },
    # "locus":{
    #     "session_name": "locus_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/locus.yaml",
    #         "-d",
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/spot1/velodyne_points/transformed",
    #         "imu_topic": "/imu/data"
    #     },
    #     "output_topics": [
    #         "/spot1/locus/odometry",
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     ]
    # },

    # "color_loam":{
    #     "session_name": "color_loam_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/color_loam.yaml",
    #         "-d",
    #     ],
    #     # optional
    #     "clean_command":[
    #         "bash",
    #         "clean_scripts/color_loam.sh"
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/microstrain/imu/data"
    #     },
    #     "output_topics": [
    #         "/odometry",
    #         "/execution_time"
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     #    "-u 10"
    #     ]
    # },
    # "color_loam2":{
    #     "session_name": "color_loam2_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/color_loam2.yaml",
    #         "-d",
    #     ],
    #     # optional
    #     "clean_command":[
    #         "bash",
    #         "clean_scripts/color_loam2.sh"
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/microstrain/imu/data"
    #     },
    #     "output_topics": [
    #         "/odometry",
    #         "/execution_time"
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     #    "-u 10"
    #     ]
    # },
    # "color_loam3":{
    #     "session_name": "color_loam3_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/color_loam3.yaml",
    #         "-d",
    #     ],
    #     # optional
    #     "clean_command":[
    #         "bash",
    #         "clean_scripts/color_loam3.sh"
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/microstrain/imu/data"
    #     },
    #     "output_topics": [
    #         "/odometry",
    #         "/execution_time"
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     #    "-u 10"
    #     ]
    # },
    # "color_loam4":{
    #     "session_name": "color_loam4_session",
    #     "launch_command": [
    #         "tmuxp",
    #         "load",
    #         "launch_scripts/color_loam4.yaml",
    #         "-d",
    #     ],
    #     # optional
    #     "clean_command":[
    #         "bash",
    #         "clean_scripts/color_loam4.sh"
    #     ],
    #     "input_topics": {
    #         "lidar_topic": "/velodyne_points",
    #         "imu_topic": "/microstrain/imu/data"
    #     },
    #     "output_topics": [
    #         "/odometry",
    #         "/execution_time"
    #     ],
    #     "rosbag_args": [
    #         "--clock",
    #     #    "-u 10"
    #     ]
    # },
}


# colors
RED     = "\033[1;31m"  
BLUE    = "\033[1;34m"
CYAN    = "\033[1;36m"
GREEN   = "\033[0;32m"
ORANGE  = "\033[93m"
RESET   = "\033[0;0m"
BOLD    = "\033[;1m"
REVERSE = "\033[;7m"


# waiting time in seconds before and after rosbag play
WAITING_TIME = 10
