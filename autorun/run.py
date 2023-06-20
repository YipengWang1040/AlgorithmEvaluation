import subprocess
import argparse
import os
import sys
import shutil
import signal
import time
import traceback
import json

import rosbag

from definations import *



# wrappers for subprocess.Popen

# There are 3 subprocesses in total:
#   1. 'rosbag play' subprocess. This process is also used to indicate when the run ends and all other processes should shutdown.
#   2. 'rosbag record' subprocess. This is used to record the output odometry and execution time.
#   3. algorithm subprocess. Anything related to the algorithm should be packed into one tmuxp config.

def create_algorithm_process(args):
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def destroy_algorithm_process(session):
    subprocess.run(args=["tmux","kill-session","-t", session], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def create_rosbag_play_process(location, sequence, lidar_topic, lidar_topic_remap, imu_topic, imu_topic_remap, additional_args, waiting_time):
    if(os.path.exists(location+"/"+sequence+".bag")):
        process = subprocess.Popen(args=" ".join(["rosbag", "play", "-d", str(waiting_time),
                                                sequence+".bag", 
                                                "{}:={}".format(lidar_topic, lidar_topic_remap),
                                                "{}:={}".format(imu_topic, imu_topic_remap),
                                                ] + additional_args),
                                    cwd=location, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        process = subprocess.Popen(args=" ".join(["rosbag", "play", "-d", str(waiting_time),
                                                sequence+"/*.bag", 
                                                "{}:={}".format(lidar_topic, lidar_topic_remap),
                                                "{}:={}".format(imu_topic, imu_topic_remap),
                                                ] + additional_args),
                                    cwd=location, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    return process

# https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/?answer=31653#post-id-31653
def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    # assert retcode == 0, "ps command returned %d" % retcode
    if(retcode != 0):
        return
    for pid_str in ps_output.split(b"\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    p.terminate()

def destroy_rosbag_play_process(process):
    terminate_process_and_children(process)

def create_rosbag_record_process(topics, location):
    process = subprocess.Popen(args=" ".join(["rosbag", "record"] + topics +
                                             ["-O", location+"/results.bag"]), 
                                shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return process

def destroy_rosbag_record_process(process):
    terminate_process_and_children(process)

# optional cleaning process
def clean(args):
    return subprocess.run(args)

def estimate_time_usage(datasets_all, dataset_used, waiting_time):
    # load execution time from file, if existed
    if(os.path.exists("cached_datasets.json")):
        with open("cached_datasets.json", 'r') as f:
            js = json.load(f)
            if(js["datasets"]==json.dumps(datasets_all)):
                print("  Loaded time usage for this dataset.")
                execution_time = 0
                for dataset in js["map"]:
                    for sequence in js["map"][dataset]:
                        if(dataset in dataset_used and sequence in dataset_used[dataset]["sequences"]):
                            execution_time += js["map"][dataset][sequence] + 2 * waiting_time
                return execution_time, js["map"]
               
    execution_time = 0
    time_map = {}
    for dataset in datasets_all:
        dataset_config = datasets_all[dataset]
        time_map[dataset] = {}
        for sequence in dataset_config["sequences"]:
            sequence_time = 0
            if(os.path.exists(dataset_config["location"]+"/"+sequence+".bag")):
                bag=rosbag.Bag(dataset_config["location"]+"/"+sequence+".bag",'r')
                sequence_time += bag.get_end_time()-bag.get_start_time()
            else:
                directory = dataset_config["location"] + '/' + sequence
                bags=[]
                with os.scandir(directory) as dir:
                    for entry in dir:
                        if(entry.name.endswith(".bag")):
                            bags.append(entry.name)
                bags.sort()
                for bag_path in bags:
                    bag=rosbag.Bag(directory+'/'+bag_path,'r')
                    sequence_time += bag.get_end_time()-bag.get_start_time()
        
            time_map[dataset][sequence] = sequence_time
            if(dataset in dataset_used and sequence in dataset_used[dataset]["sequences"]):
                execution_time += sequence_time + 2 * waiting_time


    js={
        "datasets": json.dumps(datasets_all),
        "map": time_map
    }
    with open("cached_datasets.json", 'w') as f:
        json.dump(js, f)
            
    return execution_time, time_map
    

def main():
    parser = argparse.ArgumentParser("Automatically run all datasets on all algorithms and collect the results.")
    parser.add_argument("--algorithms", help="specify only certain algorithms to run.", nargs="*", default=[])
    parser.add_argument("--datasets", help="specify only certain datasets to run.", nargs="*", default=[])
    parser.add_argument("-c", "--continue", help="continue mode. Try to continue the previous unfinished run.", default=None, dest="continue_")
    parser.add_argument("--name", help="specify name of output folder.", default=None)
    args = parser.parse_args()

    algorithms = {}
    for algorithm in args.algorithms:
        if algorithm in ALGORITHMS:
            algorithms[algorithm] = ALGORITHMS[algorithm]
        else:
            print("Skipping invalid algorithm {}{}{} specified in the arguments.".format(GREEN,algorithm,RESET))

    if(len(algorithms)==0):
        algorithms = ALGORITHMS
    
    datasets = {}
    for dataset in args.datasets:
        if dataset in DATASETS:
            datasets[dataset] = DATASETS[dataset]
        else:
            print("Skipping invalid dataset {}{}{} specified in the arguments.".format(BLUE,dataset,RESET))

    if(len(datasets)==0):
        datasets = DATASETS

    waiting_time = WAITING_TIME

    if(args.continue_ is not None):
        # load progress map
        results_dir = args.continue_
        progress_file = results_dir + "/progress.json"
        if(not os.path.exists(progress_file)):
            print("Unable to continue previous unfinished job \"{}\". Job not exist or finished.".format(results_dir))
            return -1
        
        with open(progress_file) as f:
            print("Loaded previous unfinished job \"{}\". Finished part will be skipped.".format(results_dir))
            progress = json.load(f)
        
        # load running configs
        with open(results_dir + '/results.json') as f:
            definations = json.load(f)
            datasets = definations["datasets"]
            algorithms = definations["algorithms"]
            waiting_time = definations["waiting_time"]
        try:
            total_time, time_map = estimate_time_usage(DATASETS, datasets, waiting_time)
        except:
            print("Corrupted dataset cache. Please remove \"cached_datasets.json\" and try again.")
            return -1
        
        # verify dataset and update running time
        finished_time = 0
        for algorithm in algorithms:
            for dataset in datasets:
                for sequence in datasets[dataset]["sequences"]:
                    if(algorithm not in ALGORITHMS):
                        print("Algorithm {}{}{} loaded but not in defination."
                               "Please make sure the definations are compatible with the unfinished job.".format(GREEN,algorithm,RESET))
                        return -1
                    if(dataset not in DATASETS):
                        print("Dataset {}{}{} loaded but not in defination."
                               "Please make sure the definations are compatible with the unfinished job.".format(BLUE,dataset,RESET))
                        return -1
                    if(sequence not in DATASETS[dataset]["sequences"]):
                        print("Sequence {}{}{} in dataset {}{}{} loaded but not in defination."
                               "Please make sure the definations are compatible with the unfinished job.".format(CYAN,sequence,RESET,BLUE,dataset,RESET))
                        return -1
                    
                    if(progress[algorithm][dataset][sequence]==1):
                        finished_time += time_map[dataset][sequence] + 2 * waiting_time

        print("Estimated running time is {}{:.3f}{} seconds".format(ORANGE, total_time * len(algorithms) - finished_time, RESET)) 
        
    else:
        results_dir="results"
        if(args.name is None):
            results_idx=0
            while(os.path.exists(results_dir)):
                results_idx += 1
                results_dir = "results_%d" % results_idx
        else:
            results_dir="results_"+args.name
            if(os.path.exists(results_dir)):
                print("Result directory existed. Do you want to remove it? (y/n)")
                confirm=input()
                if(confirm=='y' or confirm=='Y' or confirm=='yes' or confirm=='Yes'):
                    shutil.rmtree(results_dir)
                else:
                    return -1
        os.mkdir(results_dir)
        print("Saving results to \"{}\"".format(os.path.abspath(results_dir)))
        with open(results_dir + '/results.json', 'w') as f:
            definations = {}
            definations["datasets"] = datasets
            definations["algorithms"] = algorithms
            definations["waiting_time"] = waiting_time # no need to save this. Better just modify the source code
            json.dump(definations, f)

        try:
            total_time, time_map = estimate_time_usage(DATASETS, datasets, waiting_time)
        except:
            print("Corrupted dataset cache. Please remove \"cached_datasets.json\" and try again.")
            return -1
        print("Estimated running time is {}{:.3f}{} seconds".format(ORANGE,total_time * len(algorithms),RESET)) 

        # create progress map
        progress = {}
        for algorithm in algorithms:
            progress[algorithm]={}
            for dataset in datasets:
                progress[algorithm][dataset]={}
                for sequence in datasets[dataset]["sequences"]:
                    progress[algorithm][dataset][sequence] = 0

        progress_file = results_dir + "/progress.json"
        with open(progress_file, 'w') as f:
            json.dump(progress, f)

    program_start = time.time()


    for algorithm in algorithms:
        algorithm_start = time.time()
        print("Processing algorithm \"{}{}{}\"".format(GREEN,algorithm,RESET))
        algorithm_config = algorithms[algorithm]
        algorithm_dir = results_dir+"/"+algorithm+"_results"
        if(not os.path.isdir(algorithm_dir)):
            os.mkdir(algorithm_dir)

        # find the remap lidar and imu topic
        lidar_topic_remap = algorithm_config["input_topics"]["lidar_topic"]
        imu_topic_remap = algorithm_config["input_topics"]["imu_topic"]

        for dataset in datasets:
            print("  Processing dataset \"{}{}{}\"".format(BLUE,dataset,RESET))
            dataset_config = datasets[dataset]
            dataset_dir = algorithm_dir + "/" + dataset
            if(not os.path.isdir(dataset_dir)):
                os.mkdir(dataset_dir)

            sequences_time = time_map[dataset]
            
            location = dataset_config["location"]
            lidar_topic = dataset_config["lidar_topic"]
            imu_topic = dataset_config["imu_topic"]
            for sequence in dataset_config["sequences"]:
                if(progress[algorithm][dataset][sequence]==1):
                    print("    Skipped finished sequence \"{}{}{}\".".format(CYAN,sequence,RESET))
                    continue

                print("    Processing sequence \"{}{}{}\". Waiting for the algorithm...".format(CYAN,sequence,RESET), end='\r')
                sequence_dir = dataset_dir + "/" + sequence

                # for robustness
                if (os.path.isdir(sequence_dir)):
                    shutil.rmtree(sequence_dir)
                    
                os.mkdir(sequence_dir)
                try:
                    algorithm_process = create_algorithm_process(algorithm_config["launch_command"])
                    record_process = create_rosbag_record_process(algorithm_config["output_topics"], sequence_dir)
                    play_process = create_rosbag_play_process(location, sequence, lidar_topic, lidar_topic_remap, imu_topic, imu_topic_remap, algorithm_config["rosbag_args"], waiting_time)
                    time.sleep(waiting_time)
                    print("    Processing sequence \"{}{}{}\". Algorithm running...        ".format(CYAN,sequence,RESET))
                    sequence_start = time.time()
                    while(play_process.poll() is None):
                        print("      Estimated progress: {:.3f} / {:.3f}".format(time.time() - sequence_start, sequences_time[sequence]),end = '\r', flush=True)
                        time.sleep(0.016)  # 60 fps!
                    print("    Done. Waiting for other unfinished jobs...", end='\r')
                    
                    time.sleep(waiting_time)
                    destroy_algorithm_process(algorithm_config["session_name"])
                    destroy_rosbag_record_process(record_process)
                    if("clean_command" in algorithm_config):
                        clean(algorithm_config["clean_command"]+[sequence_dir])
                    print("    Done.                                             ")

                    progress[algorithm][dataset][sequence]=1
                    with open(progress_file, 'w') as f:
                        json.dump(progress, f)
                except:
                    print(traceback.format_exc())
                    destroy_algorithm_process(algorithm_config["session_name"])
                    destroy_rosbag_record_process(record_process)
                    destroy_rosbag_play_process(play_process)
                    if("clean_command" in algorithm_config):
                        clean(algorithm_config["clean_command"]+[sequence_dir])

                    print("\n\n\nCancelled after {:.3f} seconds.".format(time.time()-program_start))
                    return -1

                    
        
        print("Algorithm \"{}{}{}\" ends after {:.3f} seconds".format(GREEN,algorithm,RESET, time.time()-algorithm_start))

    print("\n\n\nTotal running time {:.3f}".format(time.time()-program_start))
    # remove progress file
    os.remove(progress_file)
    

if __name__ == "__main__":
    sys.exit(main())
