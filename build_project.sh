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

catkin_make -DCMAKE_BUILD_TYPE=Debug

echo "Sourcing the workspace..."
source $PROJECT_PATH/devel/setup.bash
echo "Finished sourcing the workspace"
