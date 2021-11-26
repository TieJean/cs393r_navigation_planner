import yaml
import os
from math import hypot
import in_place
from enum import Enum

# BAG_YAML_DIR = "../bag/GDC1/"
BAG_YAML_DIR = "../"
LUA_DIR = "../config/"
LUA_FILENAME = "particle_filter.lua"

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

# returns an array of tuples, where each value in the tuple is a dictionary with data for 1 timestamp
def yaml2dict(filename):
    ret = []
    with open(os.path.join(BAG_YAML_DIR, filename), "r") as stream:
        try:
            dictionaries = yaml.safe_load_all(stream)
            for dictionary in dictionaries:
                ret.append(dictionary)
        except yaml.YAMLError as exc:
            print(exc)
    return ret

def parse_bag_yaml(filename):
    ret = []
    with open(os.path.join(BAG_YAML_DIR, filename), "r") as stream1:
        with open(os.path.join(BAG_YAML_DIR, "pf_" + filename), "r") as stream2:
            # print(filename)
            # print("pf_" + filename)
            try:
                dictionaries1 = yaml.safe_load_all(stream1)
                dictionaries2 = yaml.safe_load_all(stream2)
                for dictionary in zip(dictionaries1, dictionaries2):
                    ret.append(dictionary)
            except yaml.YAMLError as exc:
                print(exc)
    return ret

# returns the average Euclidean distance between every DOWNSAMPLE_RATE points
def compute_distance(locs):
    DOWNSAMPLE_RATE = 10
    total_dist = 0
    count = 0
    for val in locs:
        if val[0] is not None and val[1] is not None and count % DOWNSAMPLE_RATE == 0:
            # print(val)
            total_dist += hypot(val[0]['pose']['x']-val[1]['pose']['x'], val[0]['pose']['y']-val[1]['pose']['y'])
        count += 1
    return total_dist / (count // DOWNSAMPLE_RATE)

# changes the parameters in the particle_filter.lua file
# def change_params(num_particles, obs_std, d_long, d_short, motion_dist, a_k1, a_k2):
def change_params(obs_std):
    # TODO add more params once this is confirmed working
    with in_place.InPlace(LUA_DIR + LUA_FILENAME) as fp:
        for line in fp:
            vars = line.split()
            if len(vars) != 3:
                fp.write(line)
                continue
            var = vars[0]
            if var == "SENSOR_STD_DEV":
                new_line = var + " = " + str(obs_std) + "\n"
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
    obstacle = int(filename[3].split('.')[0]) == 1  # dense: 1, sparse: 0 and 2
    
    if (context == Context.LS_LM_LO):
        return not laser and not (drift or error) and not obstacle
    elif (context == Context.LS_LM_HO):
        return not laser and not (drift or error) and obstacle
    elif (context == Context.LS_LM_LO):
        return not laser and (drift or error) and not obstacle
    elif (context == Context.LS_LM_LO):
        return not laser and (drift or error) and obstacle
    elif (context == Context.LS_LM_LO):
        return laser and not (drift or error) and not obstacle
    elif (context == Context.LS_LM_HO):
        return laser and not (drift or error) and obstacle
    elif (context == Context.LS_LM_LO):
        return laser and (drift or error) and not obstacle
    elif (context == Context.LS_LM_LO):
        return laser and (drift or error) and obstacle
    else:
        return False

# finds the best parameters for a particular bag file
# context is a dictionary with cutoffs for each noise level
# NOTE: ASSUMES THE YAML FILE FOR THE GROUND TRUTH ALREADY EXISTS
'''
context: {
    laser_noise_cutoff: float, 
    angular_drift_cutoff: float, 
    angular_error_cutoff: float, 
    obstacle_map: int
    }
'''
def find_best_params(context):
    # set1 = [50, 0.05, ...]
    # num_particles = [50, 60, 70]
    obs_std = [0.05, 0.1, 0.15]
    # d_long = []
    # d_short = []
    # motion_dist = []
    # a_k1 = []
    # a_k2 = []
    
    best_dist = 10000000
    best_params = []
    for std in obs_std:
        dist = 0
        for filename in os.listdir(BAG_YAML_DIR):
            # determine if this file belongs to this context
            if (not filename.endswith(".bag")) or filename[:2] == "pf" or not file_in_context(filename, context):
                continue
            
            change_params(std) # TODO: add more params
            os.system("./generate_pf_file.sh -f " + filename) # TODO: Change this method to accept a single bag file
            os.system("./read_rosbag.sh -f " + filename) # generates yaml TODO: change this too
            locs = parse_bag_yaml(filename + ".yaml")
            dist += compute_distance(locs)

        if dist < best_dist:
            best_dist = dist
            best_params = [std]

    return best_params

def find_best_params_all_contexts():
    '''
    Find the best params for each context.
    '''
    file = open("best_params.txt", "w")
    for context in Context:
        file.write(context.name + "\n")
        best_params = find_best_params(context)
        file.write(str(best_params) + "\n\n")
    file.close()
    return

if __name__ == '__main__':
    find_best_params_all_contexts()

    # for filename in os.listdir(BAG_YAML_DIR):
    #     if (not filename.endswith(".yaml")) or filename[:8] == "filtered" or filename[:2] == "pf":
    #         continue
    #     dictionaries = parse_bag_yaml(filename)
    # print(len(dictionaries))
    # dictionary = dictionaries[0]
    # print(dictionary[0]["pose"]["x"], dictionary[0]["pose"]["y"], dictionary[0]["pose"]["theta"])
    # print(dictionary[1]["pose"]["x"], dictionary[1]["pose"]["y"], dictionary[1]["pose"]["theta"])
    # diff = []
    # for dictionary in dictionaries:
    #     # print(dictionary)
    #     if (dictionary[0] and dictionary[1]):
    #         actual = dictionary[0]["pose"]
    #         predicted = dictionary[1]["pose"]
    #         diff.append((actual["x"] - predicted["x"], actual["y"] - predicted["y"], actual["theta"] - predicted["theta"]))
    #         # diff.append((dictionary[0]["pose"]["x"] - dictionary[1]["pose"]["x"], dictionary[0]["pose"]["y"] - dictionary[1]["pose"]["y"], dictionary[0]["pose"]["theta"] - dictionary[1]["pose"]["theta"]))
    #         # print(actual["x"], actual["y"], actual["theta"])
    #         # print(predicted["x"], predicted["y"], predicted["theta"])
    # print(len(diff))
    # context = {
    #     'laser_noise_cutoff': 1, 
    #     'angular_drift_cutoff': 1, 
    #     'angular_error_cutoff': 1, 
    #     'obstacle_map': 0
    # }
    # print(find_best_params(context))

