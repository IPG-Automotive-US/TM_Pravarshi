#!/bin/bash

# get list of gpus on the node
readarray -t devices <<< $(nvidia-smi --query-gpu=name --format=csv)

source ./ros/ros1_ws/devel/setup.bash

./bin/TruckMaker.linux64 -ngpu 2 -screen -dstore ${FileName} &

# Start TM
./Start_GPU.sh
