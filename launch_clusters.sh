#!/bin/bash

declare -a LIMBS=(
    "roslaunch motor_controller left_leg_cluster_node.launch"
    "roslaunch motor_controller left_arm_cluster_node.launch"
    "roslaunch motor_controller right_leg_cluster_node.launch"
    "roslaunch motor_controller right_arm_cluster_node.launch"
)

for CMD in "${LIMBS[@]}"; do 
    gnome-terminal -- bash -c "$CMD; exec bash"
done