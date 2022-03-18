# 5G-ERA Reference NetApp Workshop

Before start of this workshop, please be sure, that you complete all steps from [Prerequisities](Documentation/0_Prerequisites.md).

## Introduction

### Update repository and build ROS2 5G-ERA Reference NetApp
```bash
cd ~/NetApp-Workshop/
git pull
cd NetApp_ros2_src/
colcon build
```

### Update .bashrc
```bash
echo "## Source workshop environment" >> ~/.bashrc
echo "source $(pwd)/install/local_setup.sh" >> ~/.bashrc
echo "## Setup environment variables" >> ~/.bashrc
echo "source $(pwd)/set_environment.sh" >> ~/.bashrc
source ~/.bashrc
```

The commands above assume that NetApp-Workshop repository is in the home directory.

## Demo - ImagePublisher / ResultSubscriber / DummyDetector
```bash
TBD
```

## Demo - ML_service / Service Call / ImagePublisher / ResultSubscriber
```bash
# Terminal 1
ros2 run ros2_5g_era_object_detection_standalone_py ml_service

# Terminal 2


# Terminal X - Service call
ros2 service call /control_service/start ros2_5g_era_service_interfaces/Start 

ros2 service call /control_service/stop ros2_5g_era_service_interfaces/Stop # TODO: Tady bude něco chybět asi
```

<!-- 
## Demo - Robot / ML_service (Host system) - BACKUP
```bash
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
-->


## Demo - Robot / ML_service (Kubernetes)
TODO: Musí se upravit!!!
```bash
# Terminal 1
cd ~/NetApp-Workshop/NetApp_k8_deploy/ #TODO: Change to NetApp_k8s_deploy ??

kubectl apply -f multus_config.yaml

kubectl apply -f 5gera_ml_service_standalone.yaml

watch "microk8s.kubectl get all"
```

```bash
# Terminal 2
watch "ros2 topic list"
```

After 
```bash
# Terminal 3
ros2 run ros2_5g_era_robot_py robot_node

# Terminal 4
## Start 
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /control_service}"

## Stop
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```

Delete deployed CSS

```bash
kubectl delete deployment.apps/ros-css-deployment
```
## Demo - Robot / ML_service - distributed (Kubernetes)

```bash
# Terminal 1
cd ~/NetApp-Workshop/NetApp_k8_deploy/ #TODO: Change to NetApp_k8s_deploy ??


kubectl apply -f 5gera_ml_service_distributed.yaml

watch "microk8s.kubectl get all"
```

```bash
# Terminal 2
watch "ros2 topic list"
```

After 
```bash
# Terminal 3
ros2 run ros2_5g_era_robot_py robot_node

# Terminal 4
## Start 
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /control_service}"

## Stop
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```

Delete all deployments and services

```bash
kubectl delete deployment.apps/ml-worker-deployment
kubectl delete deployment.apps/distributed-css-deployment
kubectl delete deployment.apps/redis-deployment
kubectl delete deployment.apps/rabbit-deployment
kubectl delete service/rabbitmq-service
kubectl delete service/redis-service
```
