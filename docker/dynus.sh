#!/bin/bash

source ~/.bashrc
source /home/kkondo/code/dynus_ws/install/setup.bash
source /home/kkondo/code/decomp_ws/install/setup.bash
source /usr/share/gazebo/setup.sh

# (0) sim in static environment
tmuxp load /home/kkondo/code/dynus_ws/src/dynus/launch/docker_dynus_sim.yaml
