#!/bin/bash/home/docker

# This script is used to set up the environment for the Dynus simulation in Docker.
source ~/.bashrc
source /home/XXXXXXX/code/dynus_ws/install/setup.bash
source /usr/share/gazebo/setup.sh

# Run the Dynus simulation
python3 src/dynus/launch/run_single_sim.py --env easy_forest
