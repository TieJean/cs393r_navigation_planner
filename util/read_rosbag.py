import yaml
import os
from math import hypot

# BAG_YAML_DIR = "../bag/GDC1/"
BAG_YAML_DIR = "../"

# returns an array of tuples, where each value in the tuple is a dictionary with data for 1 timestamp
def parse_bag_yaml(filename):
    ret = []
    with open(os.path.join(BAG_YAML_DIR, filename), "r") as stream1:
        with open(os.path.join(BAG_YAML_DIR, "pf_" + filename), "r") as stream2:
            print(filename)
            print("pf_" + filename)
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
        if count % DOWNSAMPLE_RATE == 0:
            total_dist += math.hypot(val[0]['x']-val[1]['x'], val[0]['y']-val[1]['y'])
        count += 1
    return total_dist / (count // DOWNSAMPLE_RATE)

# changes the parameters in the particle_filter.lua file
# def change_params(num_particles, obs_std, d_long, d_short, motion_dist, a_k1, a_k2):
def change_params(obs_std):
    # TODO add more params once this is confirmed working
    return

# finds the best parameters for a particular bag file
# context is a dictionary with cutoffs for each noise level
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
    
    best_dist = 1000000
    best_params = []
    for std in obs_std:
        for filename in os.listdir(BAG_YAML_DIR):
            if (not filename.endswith(".yaml")) or filename[:2] == "pf":
                continue
            # determine if this file belongs to this context


            change_params(std) # TODO: add more params
            os.system("./generate_pf_file.sh --filename", bag_filename) # TODO: Change this method to accept a single bag file
            os.system("./read_rosbag.sh --filename", bag_filename) # generates yaml TODO: change this too
            locs = parse_bag_yaml(bag_filename + ".yaml")
            dist = compute_distance(locs)

            if dist < best_dist:
                best_dist = dist
                best_params = [std]

    return best_params


if __name__ == '__main__':
    for filename in os.listdir(BAG_YAML_DIR):
        if (not filename.endswith(".yaml")) or filename[:2] == "pf":
            continue
        dictionaries = parse_bag_yaml(filename)
    print(len(dictionaries))
    # for dictionary in dictionaries:
    #     print(dictionary)