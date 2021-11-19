import os
import in_place

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
    laser_noise_stddev = [0, 0.01, 0.02, 0.05, 0.07, 0.1]
    angular_drift_rate = [0, 0.2, 0.3, 0.1, 1.5, 1.5]
    angular_error_rate = [0, 5.0, 7.0, 25.0, 5.0, 25.0]

    for laser_noise in laser_noise_stddev:
        for i in range(0, len(angular_drift_rate)):
            change_lua(laser_noise, angular_drift_rate[i], angular_error_rate[i])
            # call rosbag play to record file
            file_name = "laser" + laser_noise + "_drift" + angular_drift_rate[i] + "_error" + angular_error_rate[i]
            os.chdir(UT_AUTOMATA_DIR)
            os.system(UT_AUTOMATA_DIR + "/bin/simulator")
            os.chdir(PARTICLE_FILTER_DIR)
            os.system("bin/navigation")
            os.chdir(PARTICLE_FILTER_DIR + "bag/")
            os.system("rosbag record -a -O " + file_name + " --duration=15")
            sleep(20)
            

if __name__ == '__main__':
    PARTICLE_FILTER_DIR = os.chdir("../") + "/"
    # change_lua(1.0, 2.0, 3.0)
    os.chdir(UT_AUTOMATA_DIR)
    os.system("./bin/simulator")
    