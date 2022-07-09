#!/bin/bash

cd ~/ros2_foxy

if [ -z "$1" ]
then
    echo "Usage:"
    echo "./bobik_build.sh robot"
    echo "./bobik_build.sh bridge"
    echo "./bobik_build.sh arduino"
    echo "./bobik_build.sh web"
    echo "./bobik_build.sh interfaces"
    echo "./bobik_build.sh description"
    echo "./bobik_build.sh gazebo"
else
    colcon build --packages-select bobik_$1
fi

