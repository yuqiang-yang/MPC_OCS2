#!/bin/bash          
source devel/setup.bash
rosservice call /voxblox_node/load_map "file_path: '/home/vision/yq_ws/perceptive_mpc_ws/src/perceptive_mpc/example/example_map.esdf'"
