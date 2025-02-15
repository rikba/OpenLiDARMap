#!/bin/bash

# Check correct call of script
if [ $# -ne 10 ]; then
    echo "Usage: $0 <map_path> <scan_path> <output_path> <x> <y> <z> <qx> <qy> <qz> <qw>"
    echo "Example: $0 map.pcd scans/ output.txt 0 0 0 0 0 0 1"
    exit 1
fi

MAP_PATH=$1
PCD_PATH=$2
OUTPUT_PATH=$3
X=$4
Y=$5
Z=$6
QX=$7
QY=$8
QZ=$9
QW=${10}

tag=0.2.0

xhost +
docker run --rm -it \
    --network=host \
    -v /dev/shm:/dev/shm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    --privileged \
    -v $MAP_PATH:/map_path.pcd \
    -v $PCD_PATH:/pcd_path \
    -v $OUTPUT_PATH:/output_path.txt \
    ga58lar/openlidarmap:$tag \
    bash -c "cd /dev_ws/build && ./openlidarmap_cpp /map_path.pcd /pcd_path /output_path.txt $X $Y $Z $QX $QY $QZ $QW"
xhost -
