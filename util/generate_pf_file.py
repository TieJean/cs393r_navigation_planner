import os
import in_place
import subprocess
import signal
import time


def generate_pf_file():
    for filename in os.listdir(BAG_DIR):
        if not filename.endswith(".bag"):
            continue
        proc_pf = subprocess.Popen([PARTICLE_FILTER_DIR + "bin/particle_filter", "&"])
        os.chdir(BAG_DIR)
        new_filename = "pf_" + filename
        os.system("rosbag record localization odom scan -O " + new_filename + " --duration=5 &") # TODO
        try:
            outs, errs = proc_pf.communicate(timeout=5) # TODO
            os.chdir(BAG_DIR)
            os.system("rosbag play " + filename + " &")
        except subprocess.TimeoutExpired:
            proc_pf.kill()
        time.sleep(5)


if __name__ == '__main__':
    os.chdir("../")
    PARTICLE_FILTER_DIR = str(os.getcwd()) + "/"
    BAG_DIR = PARTICLE_FILTER_DIR
    # BAG_DIR = PARTICLE_FILTER_DIR + "bag/"

    generate_pf_file()