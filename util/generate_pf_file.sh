#!/bin/bash
prefix="auto_"

cd ..
PF_DIR=$(pwd)
cd bag
BAG_DIR=$(pwd)

cd $BAG_DIR
filenames="*.bag"
while getopts "f:p:" opt
do
    case "$opt" in
        f) filenames=${OPTARG};;
        p) prefix=${OPTARG};;
    esac
done

for f in $filenames
do
    rm -f tmp.bag
    rosbag filter $f tmp.bag "topic!='localization'"
    sleep 1
    (rosbag record localization odom scan -O $prefix$f --duration=30) &
    cd $PF_DIR
    (./bin/particle_filter) &
    pf_pid=$!
    cd $BAG_DIR
    rosbag play tmp.bag
    kill -9 $pf_pid
    ps -aux | grep "particle_filter"
    rm tmp.bag
    sleep 1
done