#!/bin/bash


cd ..
PF_DIR=$(pwd)

filenames="*.bag"
prefix="pf_"

cd bag
bag_dir=$(pwd)
while getopts "f:p:d:" opt
do
    case "$opt" in
        f) filenames=${OPTARG};;
        p) prefix=${OPTARG};;
        d) bag_dir=$PF_DIR/${OPTARG};;
    esac
done

cd $bag_dir

for f in $filenames
do
    rm -f tmp.bag
    rosbag filter $f tmp.bag "topic!='localization'"
    sleep 1
    (rosbag record localization odom scan -O $prefix$f --duration=30) &
    cd $PF_DIR
    (./bin/particle_filter) &
    pf_pid=$!
    cd $bag_dir
    rosbag play tmp.bag
    kill -9 $pf_pid
    ps -aux | grep "particle_filter"
    rm tmp.bag
    sleep 1
done