#!/usr/bin/env bash
cd ~/repos/2021-tfm-pedro-arias/follow_color
mkdir build
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$PWD/build
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/models
cd build
rm -rf *
cmake ../
make
cd ~/catkin_ws
catkin build follow_color
cd ~/repos/2021-tfm-pedro-arias/follow_color