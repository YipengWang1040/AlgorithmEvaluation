import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import yaml
import tf.transformations
import numpy as np

import curses

from ate import ate
from rpe import rpe
from utils import calculate_path_distance, path_statistics


import
class history:
    def __init__(self, buffer_size = 100) -> None:
        self.buffer=[None]*buffer_size
        self.tail = 0
        self.size = buffer_size

    def add(self, str):
        self.buffer[self.tail]=str
        self.tail = (self.tail + 1) % self.size
    
    def get(self, depth):
        if(depth<=0):
            return self.buffer[(self.tail-1) % self.size], 1
        if(depth>=self.size):
            return self.buffer[self.tail], self.size
        
        return self.buffer[(self.tail-depth)%self.size], depth


stdscr:curses.window = None
def initialize_curses():
    global stdscr
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)

def finalize_curses():
    global stdscr
    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()

def input(gt, history):
    buffer = []
    while(True):
        key = stdscr.getch()
        if(key == curses.KEY_ENTER):
            return "".join(buffer)
        elif(key == curses.KEY_BACKSPACE):
            if(len(buffer))
    pass

def display_commands():
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()
    stdscr.addnstr(0,0,"=====================================================================================", max_x)
    stdscr.addnstr(1,0,"ls:                     list all datasets", max_x)
    stdscr.addnstr(2,0,"ls DATASET:             list all sequences of the given dataset", max_x)
    stdscr.addnstr(3,0,"set DATASET SEQUENCE:   set the dataset and sequence to be displayed", max_x)
    stdscr.addnstr(4,0,"information:            display the information of the displayed dataset and sequence", max_x)
    stdscr.addnstr(5,0,"end:                    exit visualization mode", max_x)
    
    stdscr.move(max_y,0)

def list(commands, gt):
    if(len(commands)==1):
        for dataset in gt.keys():
            print("{}{}{}:".format(BLUE, dataset, RESET))
            for sequence in gt[dataset].keys():
                print("  {}{}{}".format(CYAN, sequence, RESET))
    elif(len(commands)==2):
        if(commands[1] in gt):
            print("{}{}{}:".format(BLUE, commands[1], RESET))
            for sequence in gt[commands[1]].keys():
                print("  {}{}{}".format(CYAN, sequence, RESET))
        else:
            print("{}Invalid dataset {}{}{}!{}".format(RED, RESET, commands[1], RED, RESET))
    else:
        print("{}Invalid syntax \"{}{}{}\"!{}".format(RED, RESET, ' '.join(commands),RED, RESET))

def set_current(commands, results, gt, publishers):
    if(len(commands)!=3):
        print("{}Invalid syntax \"{}{}{}\"!{}".format(RED, RESET, ' '.join(commands),RED, RESET))
        return False
    dataset = commands[1]
    sequence = commands[2]
    if(not dataset in gt):
        print("{}Invalid dataset \"{}{}{}\"!{}".format(RED, RESET, dataset,RED, RESET))
        print("Valid datasets are:")
        list(["ls"], gt)
        return False
    if(not sequence in gt[dataset]):
        print("{}Invalid sequence \"{}{}{}\"!{}".format(RED, RESET, sequence,RED, RESET))
        print("Valid sequence in {}{}{} are:".format(GREEN, dataset, RESET))
        list(["ls", dataset],gt)
        return False
    
    for algorithm in results.keys():
        publishers[algorithm].publish(results[algorithm][dataset][sequence])
    
    publishers["gt"].publish(gt[dataset][sequence])
    print("{}{}{}.{}{}{} is the current dataset. Path published.".format(BLUE,dataset,RESET,CYAN,sequence,RESET))
    
    return True

def information(commands, results, gt):
    if(commands[0] == None or commands[1] == None):
        print("Please set a valid dataset and sequence to display information!")
        return
    
    dataset = commands[0]
    sequence = commands[1]
    distance = calculate_path_distance(gt[dataset][sequence])[-1]
    max_vel, avg_vel, max_ang, avg_ang, total_dist, total_time = path_statistics(gt[dataset][sequence])
    print("Current sequence: {}{}: {}{}{}".format(GREEN, dataset, BLUE, sequence, RESET))
    print("Total distance: {:.4f} meters".format(distance))
    print("  maximum linear velocity: {:.4f} m/s".format(max_vel))
    print("  average linear velocity: {:.4f} m/s".format(avg_vel))
    print("  maximum angular velocity: {:.4f} rad/s".format(max_ang))
    print("  average angular velocity: {:.4f} rad/s".format(avg_ang))
    print("  total time: {:.4f} s".format(total_time))

    for algorithm in results:
        path = results[algorithm][dataset][sequence]
        rot, trans, trans_error = ate(path, gt[dataset][sequence], True)
        samples, sum_dist_error, sum_rotation_error = rpe(path, gt[dataset][sequence], use_time_distance=False)
        print("{}{}{}, ate: {:.6f}, rpe: {:.6f}".format(GREEN, algorithm, RESET, np.mean(trans_error), sum_dist_error/samples))
    return

def visualize(results, gt):
    rospy.init_node('results_visualizer')
    initialize_curses()

    # initialize publishers
    publishers={}
    for algorithm in results.keys():
        publishers[algorithm] = rospy.Publisher("/{}_path".format(algorithm),Path, latch=True, queue_size=10)

    publishers["gt"] = rospy.Publisher("/gt_path",Path, latch=True, queue_size=10)

    current_dataset = None
    current_sequence = None
    display_commands()
    # todo : curses interface
    while(True):
        commands = input().split()
        if(len(commands)==0):
            display_commands()
            if(current_dataset is not None):
                for algorithm in results.keys():
                    publishers[algorithm].publish(results[algorithm][current_dataset][current_sequence])
                
                publishers["gt"].publish(gt[current_dataset][current_sequence])
                print("Visualization Refreshed.")
        elif(commands[0] == "ls"):
            list(commands, gt)
        elif(commands[0] == "set"):
            if(set_current(commands, results, gt, publishers)):
                current_dataset = commands[1]
                current_sequence = commands[2]
        elif(commands[0] == "information"):
            information([current_dataset, current_sequence], results, gt)
        elif(commands[0] == "end" or commands[0] == "quit"):
            break
        else:
            print("{}Invalid syntax \"{}{}{}\"!{}".format(RED, RESET, ' '.join(commands),RED, RESET))
    
    finalize_curses()
    
    return

if __name__ == "__main__":
    visualize({},{})