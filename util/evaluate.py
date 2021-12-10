import yaml
import os
from math import hypot
import in_place
from enum import Enum
import itertools
from os.path import exists
import time
import statistics
from read_rosbag import yaml2dict, compute_distance
from generate_bag_file import generate_bagfiles
from collections import defaultdict
import numpy as np


directory_name = "bag_eval"
# EVALBAG_DIR = "../" + directory_name + "/"
LUA_DIR = "../config/"
LUA_FILENAME = "particle_filter.lua"
# PARTICLE_FILTER_DIR = "../"


def get_ground_truth():
    ground_truth = []
    for filename in os.listdir(EVALBAG_DIR):
        # parames = laser_noise_stddev, angular_drift_rate, angular_error_rate, obstacle_map
        if filename[:2] == "pf" or filename[:4] == "auto" or not filename.endswith(".bag"):
            continue
        ground_truth.append(yaml2dict(filename + ".yaml", directory=EVALBAG_DIR))
    return ground_truth


def eval(ground_truth, autotune=False):
    prefix = "auto_" if autotune else "pf_"
    
    ret = defaultdict(list)
    num_trials = 5
    index = 0
    for filename in os.listdir(EVALBAG_DIR):
        # parames = laser_noise_stddev, angular_drift_rate, angular_error_rate, obstacle_map
        if filename[:2] == "pf" or filename[:4] == "auto" or not filename.endswith(".bag") or filename.startswith("tmp"):
            continue
        
        locs_ground_truth = ground_truth[index]
        index += 1
        
        types = filename[:filename.find(".bag")].split("_")
        sensor_type = float(types[0])
        obstacle_type = float(types[3])


        for i in range(num_trials):
            os.chdir(UTIL_DIR)
            os.system("./generate_pf_file.sh -d " + directory_name + " -f " + filename + " -p " + prefix)
            os.chdir(UTIL_DIR)
            os.system("./parse_rosbag.sh -d " + directory_name + " -f " + prefix + filename)

            locs_pf = yaml2dict(EVALBAG_DIR + prefix + filename + ".yaml", directory=EVALBAG_DIR)

            dist = compute_distance(locs_ground_truth, locs_pf)
            ret[sensor_type, obstacle_type].append(dist)
        time.sleep(1)
            
    print(ret)
    for obstacle_type in ret:
        print(obstacle_type, np.mean(ret[obstacle_type]), np.std(ret[obstacle_type]), ret[obstacle_type])


if __name__ == '__main__':
    EVALBAG_DIR = os.getcwd() + "/../" + directory_name + "/"
    UTIL_DIR = os.getcwd() + "/"
    # print(EVALBAG_DIR, UTIL_DIR)

    if (False):
        generate_bagfiles(laser_noise_stddev = [0],
                      angular_drift_rate = [0],
                      angular_error_rate = [0],
                      obstacle_map = [0, 1, 2],
                      directory = directory_name)
    os.chdir(UTIL_DIR)
    if (False):
        for filename in os.listdir(EVALBAG_DIR):
            if filename[:2] == "pf" or filename[:4] == "auto" or filename.endswith(".yaml"):
                continue
            os.system(UTIL_DIR + "parse_rosbag.sh -d " + directory_name + " -f " + filename)
        
    ground_truth = get_ground_truth()
    eval(ground_truth, autotune=True)
