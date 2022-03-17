# Testing commands



First terminal - ML_service
```
source install/local_setup.sh
ros2 run ros2_5g_era_object_detection_standalone_py ml_service
```

Second terminal - Robot
```
source install/local_setup.sh
ros2 run ros2_5g_era_robot_py robot_node

```


Third terminal - Robot_control
```
source install/local_setup.sh
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /control_service}"
```
