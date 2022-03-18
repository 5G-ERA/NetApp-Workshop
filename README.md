# 5G-ERA Reference NetApp Workshop

[Prerequisities](Documentation/0_Prerequisites.md)

## Introduction

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


## Demo - Robot / ML_service (Kubernetes)
TODO: Musí se upravit!!!
```
# Terminal 1
ros2 run ros2_5g_era_object_detection_standalone_py ml_service

# Terminal 2
ros2 run ros2_5g_era_robot_py robot_node

# Terminal 3
## Start 
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /control_service}"

## Stop
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```

## Demo - Robot / ML_service - distributed (Kubernetes)