import os
import in_place
import subprocess
# import signal
import time
import random
import shutil


BAG_DIR = "new_bag/" # output directory

LUA_DIR = os.path.expanduser("~") + "/ut_automata/config/"
LUA_FILENAME = 'simulator.lua'
UT_AUTOMATA_DIR = os.path.expanduser("~") + "/ut_automata/"
MAP_DIR = os.path.expanduser("~") + "/amrl_libraries/amrl_maps/GDC1/"
MAP_FILENAME = 'GDC1.vectormap.txt'

def change_lua(laser_noise_stddev, angular_drift_rate, angular_error_rate):
    with in_place.InPlace(LUA_DIR + LUA_FILENAME) as fp:
        for line in fp:
            vars = line.split()
            if len(vars) != 3:
                fp.write(line)
                continue
            var = vars[0]
            if var == "laser_noise_stddev":
                new_line = var + " = " + str(laser_noise_stddev) + "\n"
                fp.write(new_line)
            elif var == "angular_drift_rate":
                new_line = var + " = DegToRad(" + str(angular_drift_rate) + ")\n"
                fp.write(new_line)
            elif var == "angular_error_rate":
                new_line = var + " = DegToRad(" + str(angular_error_rate) + ")\n"
                fp.write(new_line)
            else:
                fp.write(line)
    fp.close()


NO_OBSTACLES = 0
SOME_OBSTACLES = 1
MANY_OBSTACLES = 2
FULL_OBSTACLES = 3

def change_map(have_obstacles):
    os.chdir(MAP_DIR)
    original = open("Original.vectormap.txt", "r")
    obstacles = open("obstacle-map-lines.txt", "r")
    with open(MAP_FILENAME,'r+') as myfile:
        data = myfile.read()
        myfile.seek(0)
        if have_obstacles == SOME_OBSTACLES:
            obstacle_lines = list(obstacles)
            percent = 0.2
            for _ in range(0, int(len(obstacle_lines) * percent)):
                myfile.write(random.choice(obstacle_lines))
        elif have_obstacles == MANY_OBSTACLES:
            obstacle_lines = list(obstacles)
            percent = 0.5
            for _ in range(0, int(len(obstacle_lines) * percent)):
                myfile.write(random.choice(obstacle_lines))
        elif have_obstacles == FULL_OBSTACLES:
            myfile.write(obstacles.read())
        myfile.write(original.read())
        myfile.truncate()
    myfile.close()
    original.close()
    obstacles.close() 

def generate_bagfiles(laser_noise_stddev = [0, 0.02, 0.07, 0.1],
                      angular_drift_rate = [0, 0.2, 1.0, 2.5],
                      angular_error_rate = [0, 5.0, 20.0, 45.0],
                      obstacle_map = [0, 1, 2], # no obstacles, many obstacles, some obstacles
                      directory = BAG_DIR):
    for laser_noise in laser_noise_stddev:
        for i in range(0, len(angular_drift_rate)):
            for map_val in obstacle_map: # for the map conditions, 1=full obstacles, 0=no obstacles, 2=some obstacles
                change_lua(laser_noise, angular_drift_rate[i], angular_error_rate[i])
                change_map(map_val)
                os.chdir(ROOT_DIR)
                proc_nav = subprocess.Popen([ROOT_DIR + "bin/navigation", "&"])
                # proc_nav = subprocess.Popen("exec " + ROOT_DIR + "bin/navigation &", shell=True)
                os.chdir(UT_AUTOMATA_DIR)
                proc_sim = subprocess.Popen([UT_AUTOMATA_DIR + "bin/simulator", "--localize", "&"])
                # proc_sim = subprocess.Popen("exec " + UT_AUTOMATA_DIR + "bin/simulator &", shell=True)
                os.chdir(ROOT_DIR + directory + "/")
                # laser, drift, error
                filename = str(laser_noise) + "_" + str(angular_drift_rate[i]) + "_" + str(angular_error_rate[i]) + "_" + str(map_val)
                # proc_bag = subprocess.Popen(["rosbag", "record", "-a", "-O", filename, "&"])
                os.system("rosbag record localization odom scan -O " + filename + " --duration=30 &")
                # call rosbag play to record file
                try:
                    outs, errs = proc_sim.communicate(timeout=30)
                    outs, errs = proc_nav.communicate(timeout=30)
                    # outs, errs = proc_bag.communicate(timeout=15)
                except subprocess.TimeoutExpired:
                    proc_sim.kill()
                    proc_nav.kill()
                    # proc_bag.kill()
                time.sleep(5)
            

if __name__ == '__main__':
    os.chdir("../")
    ROOT_DIR = str(os.getcwd()) + "/"

    # creates map files
    # print("Creating map files ...")
    # os.chdir("./src/navigation/")
    # shutil.copy('obstacle-map-lines.txt', MAP_DIR)
    # shutil.copy(MAP_DIR + MAP_FILENAME, MAP_DIR + "Original.vectormap.txt")
    # print("Done.")

    # generates bag files
    '''
    laser_noise_stddev_max = 0.1
    laser_noise_stddev_cutoff = 0.03
    angular_drift_rate_max = 2.5  # absolute value
    angular_drift_rate_cutoff = 0.5  # absolute value
    angular_error_rate_max = 45.0  # absolute value
    angular_error_rate_cutoff = 10.0  # absolute value
    '''
    laser_noise_stddev = [0.01]
    # angular_drift_rate = [0, 0.2, 1.0, 2.5]
    # angular_error_rate = [0, 5.0, 20.0, 45.0]
    # obstacle_map = [NO_OBSTACLES, SOME_OBSTACLES, MANY_OBSTACLES, FULL_OBSTACLES]
    # laser_noise_stddev = [0]
    angular_drift_rate = [2.0]
    angular_error_rate = [30.0]
    obstacle_map = [NO_OBSTACLES]

    print("Generating bag files ...")
    generate_bagfiles(laser_noise_stddev, angular_drift_rate, angular_error_rate, obstacle_map)
    print("Done.")
    
    print("Reset map ...")
    # reset the map file
    change_map(0)
    # change_lua(1.0, 2.0, 3.0)
