# !/bin/bash

source ros_setup.bash

roslaunch rsds_camera_node SingleCam.launch &

./RSDS_Lidar_Start.sh 1
