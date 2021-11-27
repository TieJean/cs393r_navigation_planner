import yaml
import os
from math import hypot
import in_place
from enum import Enum
import itertools

# BAG_YAML_DIR = "../bag/GDC1/"
BAG_YAML_DIR = "../"
LUA_DIR = "../config/"
LUA_FILENAME = "particle_filter.lua"
CHECKPOINT = "checkpoint.txt"

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


# returns the average Euclidean distance between every DOWNSAMPLE_RATE points
def compute_distance(locs1, locs2):
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


# changes the parameters in the particle_filter.lua file
# def change_params(num_particles, obs_std, d_long, d_short, motion_dist, a_k1, a_k2):
def change_params(obs_std, d_long, d_short, motion_k1, motion_k2):
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
            elif var == "D_LONG":
                new_line = var + " = " + str(d_long) + "\n"
                fp.write(new_line)
            elif var == "D_SHORT":
                new_line = var + " = " + str(d_short) + "\n"
                fp.write(new_line)
            elif var == "MOTION_DIST_K1":
                new_line = var + " = " + str(motion_k1) + "\n"
                fp.write(new_line)
            elif var == "MOTION_DIST_K2":
                new_line = var + " = " + str(motion_k2) + "\n"
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
def find_best_params(context):
    try:
        fp = open(CHECKPOINT, 'a+')
    except IOError:
        print("error in opening checkpoint file")
        exit(1)
    lines = fp.readlines()
    if len(lines) > 0:
        checkpoint = lines[-1].split()
    # num_particles = [50, 60, 70]
    obs_std  = [0.05, 0.1, 0.15]
    d_longs  = [1.5, 2.0]
    d_shorts = [1.0]
    motion_dist_k1s = [0.1]
    motion_dist_k2s = [0.05]
    # obs_std  = [0.05, 0.1, 0.15]
    # d_longs  = [1.5, 2.0, 2.5]
    # d_shorts = [1.0, 1.5, 2.0]
    # motion_dist_k1s = [0,1, 0.3, 0.5]
    # motion_dist_k2s = [0.05, 0.15, 0.25]
    # a_k1 = []
    # a_k2 = []
    # to_print = []
    best_dist = 10000000
    best_params = []
    for std, (d_long, d_short), motion_k1, motion_k2 in \
                itertools.product(obs_std, zip(d_longs, d_shorts), motion_dist_k1s, motion_dist_k2s):
        print(len(lines))
        if len(lines) != 0:
            print(std, checkpoint[0])
            print(d_long, checkpoint[1])
            print(motion_k1, checkpoint[3])
            print(motion_k2, checkpoint[4])
            if std <= float(checkpoint[0]) or d_long <= float(checkpoint[1]) or motion_k1 <= float(checkpoint[3]) or motion_k2 <= float(checkpoint[4]):
                continue
        fp.write(str(std) + " " + str(d_long) + " " + str(d_short) + " " + str(motion_k1) + " " + str(motion_k2)) 
        d_long *= std
        d_short *= std
        dist = 0
        # for filename in os.listdir(BAG_YAML_DIR):
        #     print(filename)
        #     # determine if this file belongs to this context
        #     if (not filename.endswith(".bag")) or filename[:2] == "pf" or not file_in_context(filename, context):
        #         continue
            
        #     change_params(std, d_long, d_short, motion_k1, motion_k2) # TODO: add more params
        #     os.system("./generate_pf_file.sh -f " + filename)
        #     os.system("./read_rosbag.sh -f " + filename) # generates yaml
        #     os.system("./read_rosbag.sh -f " + "pf_" + filename) # generates yaml
        #     locs1 = yaml2dict(filename + ".yaml")
        #     locs2 = yaml2dict("pf_" + filename + ".yaml")
        #     dist += compute_distance(locs1, locs2)
        
        fp.write(" - " + str(dist) + " " + str(best_dist) + "\n")
        # if dist < best_dist:
        #     best_dist = dist
        #     best_params = [std, d_long, d_short, motion_k1, motion_k2]
    fp.close()
    # os.system("rm -f " + CHECKPOINT)
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


if __name__ == '__main__':
    file = open("best_params.txt", "w")
    best_params = find_best_params(Context.LS_LM_LO)
    file.write(str(best_params) + "\n")
    file.close()
