import yaml
import os
from math import hypot
import in_place
from enum import Enum
import itertools
from os.path import exists
import time
import statistics

BAG_YAML_DIR = "../bag/"
LUA_DIR = "../config/"
LUA_FILENAME = "particle_filter.lua"
CHECKPOINT = "./checkpoints/"

LASER_NOISE_CUTOFF = 0.03
ANGULAR_DRIFT_CUTOFF = 0.5  # absolute value
ANGULAR_ERROR_CUTOFF = 10.0 # absolute value


class Context(Enum):
    '''
    Sensor:   Low Sensor noise,      High Sensor noise
    Motion:   Low Motion noise,      High Motion noise
    Obstacle: Low (sparse) Obstacle, High (dense) Obstacle
    '''
    LS_LM_LO = 0
    LS_LM_HO = 1
    LS_HM_LO = 2
    LS_HM_HO = 3
    HS_LM_LO = 4
    HS_LM_HO = 5
    HS_HM_LO = 6
    HS_HM_HO = 7


def yaml2dict(filename, directory=BAG_YAML_DIR):
    ret = []
    with open(os.path.join(directory, filename), "r") as stream:
        try:
            dictionaries = yaml.safe_load_all(stream)
            for dictionary in dictionaries:
                ret.append(dictionary)
        except yaml.YAMLError as exc:
            print(exc)
    return ret


def compute_distance(locs1, locs2):
    '''
    Returns the average Euclidean distance between every DOWNSAMPLE_RATE points
    '''
    locs = locs1 if len(locs1) < len(locs2) else locs2 # shorter one
    other = locs1 if len(locs1) > len(locs2) else locs2

    STEP_SIZE = len(other) / len(locs)
    DOWNSAMPLE_RATE = 10
    total_dist = 0
    count = 0
    for val in locs:
        other_val = other[int(count * STEP_SIZE)]
        if val is not None and other_val is not None and count % DOWNSAMPLE_RATE == 0:
            total_dist += hypot(val['pose']['x']-other_val['pose']['x'], val['pose']['y']-other_val['pose']['y'])
        count += 1
    return total_dist / (count // DOWNSAMPLE_RATE)


def change_params(obs_std, d_long, d_short, motion_dist_k1, motion_dist_k2, motion_a_k1, motion_a_k2):
    '''
    Changes the parameters in the particle_filter.lua file
    '''
    with in_place.InPlace(LUA_DIR + LUA_FILENAME) as fp:
        for line in fp:
            vars = line.split()
            if len(vars) == 0:
                fp.write(line)
                continue
            var = vars[0]
            if var == "SENSOR_STD_DEV":
                new_line = var + " = " + str(obs_std) + "\n"
                fp.write(new_line)
            elif var == "D_LONG":
                new_line = var + " = SENSOR_STD_DEV * " + str(d_long) + "\n"
                fp.write(new_line)
            elif var == "D_SHORT":
                new_line = var + " = SENSOR_STD_DEV * " + str(d_short) + "\n"
                fp.write(new_line)
            elif var == "MOTION_DIST_K1":
                new_line = var + " = " + str(motion_dist_k1) + "\n"
                fp.write(new_line)
            elif var == "MOTION_DIST_K2":
                new_line = var + " = " + str(motion_dist_k2) + "\n"
                fp.write(new_line)
            elif var == "MOTION_A_K1":
                new_line = var + " = " + str(motion_a_k1) + "\n"
                fp.write(new_line)
            elif var == "MOTION_A_K2":
                new_line = var + " = " + str(motion_a_k2) + "\n"
                fp.write(new_line)
            else:
                fp.write(line)
    fp.close()


def file_in_context(filename, context):
    '''
    Determines if a file is in a context based on its filename.
    '''
    filename = filename.split('_')
    
    # check if the file contains certain noises
    laser = float(filename[0]) > LASER_NOISE_CUTOFF
    drift = abs(float(filename[1])) > ANGULAR_DRIFT_CUTOFF
    error = abs(float(filename[2])) > ANGULAR_ERROR_CUTOFF
    obstacle = int(filename[3].split('.')[0]) >= 2  # dense: 2 and 3, sparse: 0 and 1
    
    if (context == Context.LS_LM_LO):
        return not laser and not (drift or error) and not obstacle
    elif (context == Context.LS_LM_HO):
        return not laser and not (drift or error) and obstacle
    elif (context == Context.LS_HM_LO):
        return not laser and (drift or error) and not obstacle
    elif (context == Context.LS_HM_HO):
        return not laser and (drift or error) and obstacle
    elif (context == Context.HS_LM_LO):
        return laser and not (drift or error) and not obstacle
    elif (context == Context.HS_LM_HO):
        return laser and not (drift or error) and obstacle
    elif (context == Context.HS_HM_LO):
        return laser and (drift or error) and not obstacle
    elif (context == Context.HS_HM_HO):
        return laser and (drift or error) and obstacle
    else:
        return False


def find_best_params(context):
    '''
    Finds the best parameters for a particular context
    '''
    checkpoint = None
    best_dist = 10000000
    if (exists(CHECKPOINT + context.name + ".txt")):
        try:
            fp = open(CHECKPOINT + context.name + ".txt", 'r')
        except IOError:
            print("error in opening checkpoint file")
            exit(1)
        lines = fp.readlines()
        if len(lines) > 0:
            checkpoint = lines[-1].split()
            if len(checkpoint) > 7:
                tmp = lines[-1].split()
            else:
                tmp = lines[-2].split()
            if float(tmp[-3]) < float(tmp[-1]):
                best_dist = float(tmp[-3])
            else:
                best_dist = float(tmp[-1])
        fp.close()
    fp = open(CHECKPOINT + context.name + ".txt", 'a+')

    # obs_std  = [0.1]
    # d_shorts = [1.5]
    # d_longs  = [2.0]
    motion_dist_k1s = [0.3]
    motion_dist_k2s = [0.05]
    motion_a_k1s = [0.1]
    motion_a_k2s = [1.0]
    
    obs_std  = [0.05, 0.1, 0.15, 0.2]
    d_shorts = [1.0, 1.0, 1.5, 1.5, 2.0]
    d_longs  = [1.5, 2.0, 2.0, 2.5, 2.5]
    # motion_dist_k1s = [0.1, 0.3, 0.5]
    # motion_dist_k2s = [0.05, 0.15, 0.25]
    # motion_a_k1s = [0.1, 0.3, 0.5]
    # motion_a_k2s = [0.5, 1.0, 1.5]
    
    best_params = []

    for std, (d_long, d_short), motion_dist_k1, motion_dist_k2, motion_a_k1, motion_a_k2 in \
                itertools.product(obs_std, zip(d_longs, d_shorts), motion_dist_k1s, motion_dist_k2s, motion_a_k1s, motion_a_k2s):
        if checkpoint is not None:
            if std < float(checkpoint[0]) or d_long < float(checkpoint[1]) \
                or motion_dist_k1 < float(checkpoint[3]) or motion_dist_k2 < float(checkpoint[4]) \
                or motion_a_k1 < float(checkpoint[5]) or motion_a_k2 < float(checkpoint[6]):
                continue
        fp.write("\n" + str(std) + " " + str(d_long) + " " + str(d_short) \
                + " " + str(motion_dist_k1) + " " + str(motion_dist_k2) \
                + " " + str(motion_a_k1) + " " + str(motion_a_k2)) 
        d_long *= std
        d_short *= std
        change_params(std, d_long, d_short, motion_dist_k1, motion_dist_k2, motion_a_k1, motion_a_k2) 
        
        dist = []
        for filename in os.listdir(BAG_YAML_DIR):
            # determine if this file belongs to this context
            if (not filename.endswith(".bag")) or filename[:2] == "pf" \
                or not file_in_context(filename, context):
                continue
            os.system("./generate_pf_file.sh -f " + filename + " -p pf_")
            time.sleep(0.5)
            os.system("./parse_rosbag.sh -f " + filename) # generates yaml
            os.system("./parse_rosbag.sh -f " + "pf_" + filename) # generates yaml
            locs1 = yaml2dict(BAG_YAML_DIR + filename + ".yaml")
            locs2 = yaml2dict(BAG_YAML_DIR + "pf_" + filename + ".yaml")
            retry = 0
            while retry < 3 and (len(locs1) == 0 or len(locs2) == 0):
                if len(locs1) == 0:
                    print("retry " + filename)
                if len(locs2) == 0:
                    print("retry " + "pf_" + filename)
                os.system("./parse_rosbag.sh -f " + filename) # generates yaml
                os.system("./parse_rosbag.sh -f " + "pf_" + filename) # generates yaml
                locs1 = yaml2dict(BAG_YAML_DIR + filename + ".yaml")
                locs2 = yaml2dict(BAG_YAML_DIR + "pf_" + filename + ".yaml")
                retry += 1
            dist.append(compute_distance(locs1, locs2))
            time.sleep(1)
        
        fp.write(" - " + str(statistics.mean(dist)) + " " +  str(statistics.pstdev(dist)) + " " + str(best_dist))
        if statistics.mean(dist) < best_dist:
            best_dist = statistics.mean(dist)
            best_params = [std, d_long, d_short, motion_dist_k1, motion_dist_k2, motion_a_k1, motion_a_k2]
    fp.close()
    return best_params


def find_best_params_all_contexts():
    '''
    Find the best params for each context.
    '''
    file = open("best_params.txt", "a")
    for context in Context:
        file.write(context.name + "\n")
        best_params = find_best_params(context)
        file.write(str(best_params) + "\n\n")
    file.close()

def best_param_from_multiple_runs(context_name):
    '''
    Find best param over multiple trails
    '''
    mean = {}
    std = {}
    count = 0
    with open("checkpoints/" + context_name + ".txt", "r") as checkpoint:
        # get each param set's average mean and std over all trails 
        for line in checkpoint:
            if line == "\n":
                continue
            count += 1
            token = line.split()
            key = token[0] + "_" + token[1] + "_" + token[2]
            if key in mean:
                mean[key] += (float) (token[8])
                std[key] += (float) (token[9])
            else:
                mean[key] = (float) (token[8])
                std[key] = (float) (token[9])
    checkpoint.close()

    trails = (count / len(mean))
    best_mean = 100000.0
    best_std = 0.0
    best_key = ""
    for key in mean:
        if mean[key] / trails < best_mean:
            best_mean = mean[key] / trails
            best_std = std[key] / trails
            best_key = key
    return (best_key, best_mean, best_std)

if __name__ == '__main__':
    best_params = find_best_params(Context.LS_HM_LO)
    best_params = find_best_params(Context.LS_HM_HO)
    best_params = find_best_params(Context.HS_HM_LO)
    best_params = find_best_params(Context.HS_HM_HO)

    # file = open("best_params.txt", "w")
    # contexts = ["LS_HM_LO", "LS_HM_HO", "HS_HM_LO", "HS_HM_HO"]
    # for c in contexts:
    #     best_key, best_mean, best_std = best_param_from_multiple_runs(c)
    #     file.write(c + "\n")
    #     file.write(str(best_key) + ": " + str(best_mean) + " " + str(best_std) + "\n\n")
    # file.close()

    # locs1 = yaml2dict(BAG_YAML_DIR + "0_0_0_1.bag.yaml")
    # locs2 = yaml2dict(BAG_YAML_DIR + "auto_0_0_0_1.bag.yaml")
    # locs3 = yaml2dict(BAG_YAML_DIR + "pf_0_0_0_1.bag.yaml")
    # print("Distance for autotune", compute_distance(locs1, locs2))
    # print("Distance for no autotune", compute_distance(locs1, locs3))
