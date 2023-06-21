# AlgorithmEvaluation
Tools for automatically run and evaluate Lidar or Lidar-IMU odometry algorithms on different datasets. You can easily add your algorithms and datasets to the evaluation process.

## Overview

### autorun
`autorun/run.py` will run all algorithms defined in `autorun/definenations.py` on all datasets defined also in `autorun/definations.py` as described below:
```python
foreach ALGORITHM:
    foreach DATASET:
        foreach SEQUENCE:
            start ALGORITHM
            record output of ALGORITHM
            play SEQUENCE
            end ALGORITHM
            clean
```
The output will be stored in similar structure in a directory which can be specified by the user.
All the algorithms and datasets are defined in the file `autorun/definations.py`.
To simplify and unify the interface, all the datasets must be in `rosbag` format, and all algorithms must have a ros wrapper.
The intermediate results (e.g. the output trajectories of each algorithm) is also sotred in rosbags.

### evaluation
`evaluation/evaluate.py` will calculate the statistics (distance, speed, etc.) of each data sequence, 
and calculate the Relative Pose Error (RPE) and Absolute Trajectory Error (ATE) using given ground truth.
It can also visualize the trajectories of each algorithm V.S. the ground truth.


## Adding Your Algorithms
To add your algorithm, you have to define it in `autorun/definations.py`, add a launch script for it under `autorun/launch_scripts`, and optionally add a clean script for it.

### Definations
An algorithm is defined by adding a term in the dict `ALGORITHMS` in `autorun/definations.py`. The syntax looks like:
```json
    "ALGORITHM_NAME":{
        "session_name": "SESSION_NAME",
        "launch_command": [
            "tmuxp",
            "load",
            "launch_scripts/LAUNCH_SCRIPT.yaml",
            "-d",
        ],
        # optional
        "clean_command":[
            "bash",
            "clean_scripts/CLEAN_SCRIPT.sh"
        ],
        "input_topics": {
            "lidar_topic": "/INPUT/LIDAR/TOPIC",
            "imu_topic": "/INPUT/IMU/TOPIC"
        },
        "output_topics": [
            "/ODOMETRY_TOPIC",
            "/OTHER/OPTIONAL/TOPICS"
        ],
        "rosbag_args": [
            "--clock", # if you need it
            "OTHER ARGS"
        ]
    },
```

For example,
```json
    "ct_icp":{
        "session_name": "ct_icp_session",
        "launch_command": [
            "tmuxp",
            "load",
            "launch_scripts/ct_icp.yaml",
            "-d",
        ],
        # optional
        # "clean_command":[
        #     "bash",
        #     "clean_scripts/lego_loam.sh"
        # ],
        "input_topics": {
            "lidar_topic": "/ct_icp/pointcloud",
            "imu_topic": "/whatever"  # since CT-ICP does not use imu
        },
        "output_topics": [
            "/ct_icp/pose/odom",
            "/execution_time"
        ],
        "rosbag_args": [
            "--clock",
            "-r 0.33",
        ]
    },
```

Except for the tmuxp config file, the launch command of an algorithm should not be modified. 
The `session_name` must match the tmuxp config file, as it is used to terminate the algorithm threads. 
To utilize the evaluation scripts, one of the output topics must be the trajectory in the form of `nav_msgs/Odometry`.

### Launch Scripts
Here is an example of an launch script:
`autorun/launch_scripts/ct_icp.yaml`
```yaml
session_name: ct_icp_session
suppress_history: true
windows:
- window_name: ct_icp
  layout: tiled
  shell_command_before:
    - cd ~/ct_icp_ws/
    - source devel/setup.bash
  panes:
    - roslaunch ct_icp_odometry benchmark.launch
```
The session name must match the attribute `session_name` in the defination of the algorithm.
If you need more steps to set up the environment (e.g. modify ROS_LOG dir), you can add more commands under `shell_command_before`.
If you need other terminals to run your algorithm (e.g. launch a static transform publisher), you can add more more lines under `panes`.
You may also other windows if you need (e.g. launching a ros node in another workspace).

### Clean Scripts
Clean scripts are bash scripts that will be executed after each run of any algorithm, if provided. 
The clean script takes one argument, which is the directory the result of the current running algorithm and data sequence is stored.
The clean script can utilize it to copy some additional files (e.g. logs, configurations) to the output directory.
Below is an example of a clean script:
```bash
# beware of safety
output_dir=$1
if [ -f ~/ct_icp_ws/temp_cloud.bin ]; then
rm ~/ct_icp_ws/temp_cloud.bin
fi

if [ ! -d $output_dir/../../params.yaml ]; then
cp ~/ct_icp_ws/src/ct_icp_odometry/params/ct_icp_config.yamk $output_dir/../../params.yaml
fi
```
The above script removes a temporary file for debugging, 
then backup the configuration to the results directory. More details of the file structure can be found below. 

## Adding Your Dataset
A dataset is defined by adding a term in the dict `DATASETS` in `autorun/definations.py`. The syntax looks like:
```json
    "kitti":{
        "location":              "/media/user/external/dataset/KITTI_Odometry",
        "ground_truth":          "/home/user/dataset/kitti/ground_truth",
        "lidar_topic":           "/kitti/velodyne_points",
        "imu_topic":             "/imu",
        "sequences": [
            "00",
            "01",
            "02",
        ]
    },
```
As shown above, a dataset may contains several sequences.  A sequence can be either a single rosbag, or multiple rosbags in a folder (for splitted rosbags).
The location is the parent directory of all sequences. 

For example, in above configuration, the file structure of `/media/user/external/dataset/KITTI_Odometry` may looks like:
```
KITTI_Odometry/
├── 00.bag
├── 01
│   ├── part1.bag
│   ├── part2.bag
│   ├── part3.bag
│   └── part4.bag
└── 02.bag

```
The `ground_truth` is the parent directory of all ground truth bags. The ground truth must also be in the form of `nav_msgs/Odometry`.
For each sequence under `location`, there is expected to be a single rosbag with same name under `ground_truth` that contains the trajectory. 

If the dataset contains grount truth in the same rosbag and each sequence is not splitted, 'ground_truth' can be same as 'location', 
although for better performance, it's recommended to move ground truth topic out of the lidar data.

The 'lidar_topic' and the 'imu_topic' will be used to remap these topics for each algorithm based on its defination. Currently only single lidar is supported.

## Waiting Time
The variable `WAITING_TIME` in `autorun/definations.py` indicates how many seconds the rosbag play node will start after the algorithm process is started.
By default it is 10 seconds. If your algorithm need more time to boot, you can increase this value.

## Results
The results will be stored in such structure:
```
RESULT_DIRECTORY
├── results.json
├── algorithm1
│   ├── dataset1
│   │   └── sequence1
│   │       ├── results.bag
│   ├── dataset2
│   │   ├── sequence1
│   │   │   ├── results.bag
│   │   ├── sequence2
│   │   │   ├── results.bag
|   ........................
├── algorithm2
│   ├── dataset1
│   │   └── sequence1
│   │       ├── results.bag
│   ├── dataset2
│   │   ├── sequence1
│   │   │   ├── results.bag
│   │   ├── sequence2
│   │   │   ├── results.bag
|   ........................
├── algorithm3
............................
```
Since the structure is quite clear, relative path can always be used in the cleaning script if needed.

`results.json` contains the information of this run, which will be used later in the evaluation program. 


## Evaluation
Given the results of `autorun/run.py` in a directory called 'RESULT_DIRECTORY', run `evaluation/evaluate.py RESULTS_DIRECTORY` to evaluate the results against the ground truth. 
This requires the ground truth is correctly set. The results will also be saved to the `RESULT_DIRECTORY`. 

`--visualize` will enable an interactive visualization mode for `evaluation/evaluate.py`. 
By specifying a sequence, the visualizer will publish the trajectory of each algorithm so that they can be visualized in RViz. 
The visualizer also supports trajectory alignment, as well as ATE / RPE calculation.
