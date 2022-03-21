# Testing commands - without Kubernetes


First terminal - ML Service Worker
```
source install/local_setup.sh
cd src/ros2_5g_era_object_detection_distributed_py/
celery -A ros2_5g_era_object_detection_distributed_py.ml_service_worker worker
```


Second terminal - ML Service Interface
```
source install/local_setup.sh
ros2 run ros2_5g_era_object_detection_distributed_py ml_service
```


Third terminal - Robot
```
source install/local_setup.sh
ros2 run ros2_5g_era_robot_py robot_node

```


Fourth terminal - Robot_control
```
source install/local_setup.sh
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /control_service}"
```
