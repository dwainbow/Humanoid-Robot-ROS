#!/bin/bash

PROJECT_PATH="$HOME/Humanoid-Robot-ROS"

if [ -d "$PROJECT_PATH/build" ]; then
    echo "Removing existing build directory..."
    rm -rf "$PROJECT_PATH/build"
fi

if [ -d "$PROJECT_PATH/devel" ]; then
    echo "Removing existing devel directory..."
    rm -rf "$PROJECT_PATH/devel"
fi

catkin_make 

echo "Sourcing the workspace..."
echo $PROJECT_PAT
source "$PROJECT_PATH/devel/setup.bash"
