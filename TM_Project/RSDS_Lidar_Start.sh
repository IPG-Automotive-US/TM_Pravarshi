# !/bin/bash

if [ "$1" == "" ]; then
    echo "------------------------"
    echo "Please enter the number of lidar sensors you wish to start:"
    echo ""
    echo "      RSDS_Lidar_Start.sh <n_lidar_sensors>"
    echo ""
    echo "...also ensure you configure port numbers in the rsds_lidar_node.launch file."
    echo "------------------------"
    exit 1
fi

i=0
while [ $i -lt $1 ]; do
    echo "Starting lidar node number $i"
    ./Lidar_Helper.sh $i &
    i=$((i + 1))
done

./Start_TM.sh 
