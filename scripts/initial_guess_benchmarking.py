#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

# This script is used for the initial guess benchmarking
# 
# 
# 
# 
#  

import os
import itertools 
import subprocess
from ament_index_python.packages import get_package_share_directory


# benchmarking function
def benchmarking():
    
    # get the path to package
    package_path = get_package_share_directory('dynus')
    file_path = os.path.join(package_path, 'launch', 'initial_guess_benchmarking.launch.py')

    # create lists of parameters for the benchmarking
    global_planners_list = ["sjps", "sastar", "dastar", "dgp"]
    static_obs1_start_x_list = [-5.0, -3.0]
    static_obs2_start_x_list = [-5.0, -3.0]
    dynamic_obs_height_list = [4.0, 5.0]
    dynamic_obs_empty_space_list = [0, 1, 2, 3] # ["left" = 0, "right" = 1, "no_space" = 2, "no_obst" = 3]

    # create a combination of parameters using itertools
    combinations = list(itertools.product(static_obs1_start_x_list, static_obs2_start_x_list, dynamic_obs_height_list, dynamic_obs_empty_space_list, global_planners_list))

    # path to the data log file (needs to be the same as in dynus.yaml)
    data_log_path = "/home/kkondo/code/dynus_ws/src/dynus/data/data.txt"

    # run the benchmarking
    for static_obs1_start_x, static_obs2_start_x, dynamic_obs_height, dynamic_obs_empty_space, global_planner in combinations:
        
        # writing the benchmakring parameters in "/home/kkondo/code/dynus_ws/src/dynus/data/data.txt"
        if global_planner == global_planners_list[0]:
            with open(data_log_path, "a") as f:
                f.write("\n")
                f.write(f"static_obs1_start_x:={static_obs1_start_x}\n")
                f.write(f"static_obs2_start_x:={static_obs2_start_x}\n")
                f.write(f"dynamic_obs_height:={dynamic_obs_height}\n")
                f.write(f"dynamic_obs_empty_space:={dynamic_obs_empty_space}\n")
        
        print(f"Running benchmarking for {global_planner}")
        subprocess.run(['ros2', 'launch', file_path, f'global_planner:={global_planner}', f'static_obs1_start_x:={static_obs1_start_x}', f'static_obs2_start_x:={static_obs2_start_x}', f'dynamic_obs_height:={dynamic_obs_height}', f'dynamic_obs_empty_space:={dynamic_obs_empty_space}'])

# main function
if __name__ == "__main__":
    benchmarking()
