# 5G-ERA Reference NetApp Workshop

Before start of this workshop, please be sure, that you complete all steps from [Prerequisities](Documentation/0_Prerequisites.md).

## Introduction
```
cd ~/NetApp-Workshop/
git pull
cd NetApp_ros2_src/
colcon build
```

## Demo - ImagePublisher / ResultSubscriber / DummyDetector
```
TBD
```

## Demo - ML_service / Service Call / ImagePublisher / ResultSubscriber
```
# Terminal 1
ros2 run ros2_5g_era_object_detection_standalone_py ml_service

# Terminal 2


# Terminal X - Service call
ros2 service call /control_service/start ros2_5g_era_service_interfaces/Start 

ros2 service call /control_service/stop ros2_5g_era_service_interfaces/Stop # TODO: Tady bude něco chybět asi
```

## Demo - Robot / ML_service (Host system) - BACKUP
```
# Terminal 1
cd ~/NetApp-Workshop/NetApp_ros2_src/
source install/local_setup.sh
ros2 run ros2_5g_era_object_detection_standalone_py ml_service

# Terminal 2
cd ~/NetApp-Workshop/NetApp_ros2_src/
source install/local_setup.sh
ros2 run ros2_5g_era_robot_py robot_node

# Terminal 3
cd ~/NetApp-Workshop/NetApp_ros2_src/
source install/local_setup.sh

## Start 
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /control_service}"

## Stop
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```



## Demo - Robot / ML_service (Kubernetes)
TODO: Musí se upravit!!!
```
# Terminal 1
cd ~/NetApp-Workshop/NetApp_ros2_src/
source install/local_setup.sh
ros2 run ros2_5g_era_object_detection_standalone_py ml_service

# Terminal 2
cd ~/NetApp-Workshop/NetApp_ros2_src/
source install/local_setup.sh
ros2 run ros2_5g_era_robot_py robot_node

# Terminal 3
cd ~/NetApp-Workshop/NetApp_ros2_src/
source install/local_setup.sh

## Start 
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /control_service}"

## Stop
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```

## Demo - Robot / ML_service - distributed (Kubernetes)