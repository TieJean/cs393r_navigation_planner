#!/bin/bash
# cd ../bag/GDC1
# cd ../bag
cd ..
filenames="*.bag"
while getopts "f:" opt
do
    case "$opt" in
        f) filenames=${OPTARG};;
    esac
done

for f in $filenames
do
    (rostopic echo localization > $f.yaml) &
    topic_pid=$!
    sleep 1
    time rosbag play --immediate $f --topics localization
    sleep 5
    kill -9 $topic_pid
    echo $!
    ps -aux | grep "rostopic"
    sleep 1
done