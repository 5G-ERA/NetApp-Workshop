#!/bin/bash  
cd || exit
source dev_ws/install/local_setup.bash
ros2 run ros2_5g_era_object_detection_standalone_py ml_service
