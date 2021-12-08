#!/bin/bash

cd ..
PF_DIR=$(pwd)

filenames="*.bag"
cd bag
bag_dir=$(pwd)
while getopts "f:d:" opt
do
    case "$opt" in
        f) filenames=${OPTARG};;
        d) bag_dir=$PF_DIR/${OPTARG};;
    esac
done

echo $bag_dir
cd $bag_dir
for f in $filenames
do
    (rostopic echo localization > $f.yaml) &
    topic_pid=$!
    sleep 1
    # bash --rcfile <(echo "time rosbag play --immediate $f --topics localization")
    time rosbag play --immediate $f --topics localization
    sleep 5
    kill -2 $topic_pid
    sleep 1
    echo $!
    ps -aux | grep "rostopic"
    sleep 1
done