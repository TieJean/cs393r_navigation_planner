#!/bin/bash
# cd ../bag/GDC1
cd ../bag
for f in *.bag
do
    (rostopic echo /reference_localization > $f.yaml) &
    topic_pid=$!
    sleep 1
    time rosbag play --immediate $f --topics /reference_localization
    sleep 5
    kill -9 $topic_pid
    echo $!
    ps -aux | grep "rostopic"
    sleep 1
done