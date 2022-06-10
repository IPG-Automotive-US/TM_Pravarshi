#!/bin/bash


source ./ros/ros1_ws/devel/setup.bash

/opt/ipg/carmaker/linux64-10.0.1/GUI/MovieEGL.exe -instance 1 -mode GPUSensor -GPU 0 -exitAtSimEnd -apphost localhost -connect &

/opt/ipg/carmaker/linux64-10.0.1/GUI/MovieEGL.exe -instance 2 -mode GPUSensor -GPU 0 -exitAtSimEnd -apphost localhost -connect 


aws s3 cp /home/TM_Project/SimOutput/ ${RESULT_S3_URL} --recursive --exclude "*" --include "*.erg"
aws s3 cp /home/TM_Project/SimOutput/ ${RESULT_S3_URL} --recursive --exclude "*" --include "*.erg.info"

echo "Simulation done"
