import os
import in_place
import subprocess
import signal
import time

LUA_DIR = os.path.expanduser("~") + "/ut_automata/config/"
# LUA_FILENAME = 'simulator.lua'
LUA_FILENAME = 'test.lua'
UT_AUTOMATA_DIR = os.path.expanduser("~") + "/ut_automata/"


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

# laser_noise_stddev_max = 0.1
# laser_noise_stddev_cutoff = 0.03
# angular_drift_rate_max = 2.5  # absolute value
# angular_drift_rate_cutoff = 0.5  # absolute value
# angular_error_rate_max = 45.0  # absolute value
# angular_error_rate_cutoff = 10.0  # absolute value

def generate_bagfiles():
    laser_noise_stddev = [0]
    angular_drift_rate = [0]
    angular_error_rate = [0]
    # laser_noise_stddev = [0, 0.01, 0.02, 0.05, 0.07, 0.1]
    # angular_drift_rate = [0, 0.2, 0.3, 0.1, 1.5, 1.5]
    # angular_error_rate = [0, 5.0, 7.0, 25.0, 5.0, 25.0]

    # os.chdir(UT_AUTOMATA_DIR)
    # os.system(UT_AUTOMATA_DIR + "bin/simulator &")
    for laser_noise in laser_noise_stddev:
        for i in range(0, len(angular_drift_rate)):
            change_lua(laser_noise, angular_drift_rate[i], angular_error_rate[i])
            os.chdir(PARTICLE_FILTER_DIR)
            proc_nav = subprocess.Popen([PARTICLE_FILTER_DIR + "bin/navigation", "&"])
            # proc_nav = subprocess.Popen("exec " + PARTICLE_FILTER_DIR + "bin/navigation &", shell=True)
            os.chdir(UT_AUTOMATA_DIR)
            proc_sim = subprocess.Popen([UT_AUTOMATA_DIR + "bin/simulator", "&"])
            # proc_sim = subprocess.Popen("exec " + UT_AUTOMATA_DIR + "bin/simulator &", shell=True)
            os.chdir(PARTICLE_FILTER_DIR + "bag/")
            file_name = "laser" + laser_noise + "_drift" + angular_drift_rate[i] + "_error" + angular_error_rate[i]
            os.system("rosbag record -a -O " + file_name + " --duration=20 &")
            # call rosbag play to record file
            try:
                outs, errs = proc_sim.communicate(timeout=15)
                outs, errs = proc_nav.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                proc_sim.kill()
                proc_nav.kill()
            time.sleep(25)
            

if __name__ == '__main__':
    os.chdir("../")
    PARTICLE_FILTER_DIR = str(os.getcwd()) + "/"
    # generate_bagfiles()
    # change_lua(1.0, 2.0, 3.0)
    os.chdir(PARTICLE_FILTER_DIR)
    proc_nav = subprocess.Popen([PARTICLE_FILTER_DIR + "bin/navigation", "&"])
    os.chdir(UT_AUTOMATA_DIR)
    proc_sim = subprocess.Popen([UT_AUTOMATA_DIR + "bin/simulator", "&"])
    try:
        outs, errs = proc_sim.communicate(timeout=1)
        outs, errs = proc_nav.communicate(timeout=1)
    except subprocess.TimeoutExpired:
        print("before kill")
        os.system("ps -aux | grep \"simulator\"")
        os.system("ps -aux | grep \"navigation\"")
        time.sleep(3)
        proc_sim.kill()
        proc_nav.kill()
        print("after kill")
        print("kill -9 " + str(proc_sim.pid), "kill -9 " + str(proc_nav.pid))
        os.system("ps -aux | grep \"simulator\"")
        os.system("ps -aux | grep \"navigation\"")
    
            


    