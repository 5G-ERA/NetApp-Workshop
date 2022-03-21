# 5G-ERA Reference NetApp Workshop

Robotic platforms and automation systems require to perform complex tasks from time to time to complete their goals. These complex tasks might be computationally heavy and time-consuming when not running on suitable hardware (e.g. GPU). However, such tasks/applications might be deployed on EDGE/CLOUD with better hardware and shared between multiple robots. This workshop exploits the possibilities of running such complex tasks/applications like object detection as NetApp for robotic and automation environments.

The workshop will cover the following topics:

* Introduction to the 5G-ERA Reference NetApp
* Explanation of the NetApp Interface
* Requirements for robotic platforms
* A simple scenario in ROS2 environment
* Standalone NetApp deployment in Kubernetes
* Distributed NetApp deployment in Kubernetes

Before the start of this workshop, please be sure, that you complete all steps from [Prerequisities](Prerequisites.md).

## Introduction to the 5G-ERA Reference NetApp

The 5G-ERA Reference NetApp aims to present concept and usage of 5G-ERA Machine Learning (ML) Services. These ML services cover tasks as *object detection, object detection with movement evaluation, object classification, semantic segmentation, etc.*

These tasks may be hard to complete on robotic platforms, where EDGE/CLOUD processing takes place instead. This is exactly the place for 5G-ERA ML Services. This workshop covers demonstation usage of such ML service providing simple face detection.

In this workshop, we will omit the communication between robots and 5G-ERA Middleware, which is service responsible for correct deployment of 5G-ERA ML Services (NetApps).

### Terms definition

* **ML Service** - Machine Learning service provider (object detection, semantic segmentation, etc.)
* **ML Control Service / Control Service Server** - Server part of ROS2 service responsible for generating task UUIDs and launching new nodes inside *ML Service*.
* **ML Control Service Client / Control Service Client** - Client part of ROS2 service from previous definition. Has to be implemented by robot.
* **Robot Logic Service** - ROS2 service allowing on-demand start/stop of data processing from robot. Only for purpose of this workshop.

### Concept of 5G-ERA Reference NetApp

Two different variants of ML services are provided - **Standalone** and **Distributed** version of NetApp / ML service.

![Standalone vs. distributed concept](Images/WP4_distibuted_service_standalone_distributed.png "Standalone vs. distributed concept")

#### Standalone version

Represents concept, where complete ML service logic is integrated inside single application. There is only ROS2 communication between robots and ML service. Data flow inside application is based on internal queues (Python).

ADVANTAGES: Fast processing, latencies close to zero (data processing). Ready for image sequences dependent processing (e.g. object movement evaluation).

DISADVANTAGES: Possibly lower number of connected robots to a single instance of the ML service (hardware dependent).


#### Distributed version 

On the other hand, distributed version combines ROS2 communication with distributed data processing using Celery workers. From ML Service interface (entry point) are data for processing transmitted to workers over *Advanced Message Queuing Protocol (AMQP)*.

ADVANTAGES: Distributed processing. Possibility of scaling using more workers.

DISADVANTAGES: Slower processing, bigger latencies. A limited number of ML services suitable for distributed processing. 


**Unified communication interface** is important in concept of 5G-ERA ML Services. Different ML services requires various types of data. In our concept, communication between robots and ML service is created in ROS2. 

In animation below, you will see how is robot making request for processing data using ML service. Robot sends request through *ML Control Service* (ROS2 service) to begin communication. ML Control Service deploys new ROS2 node with dedicated data subsriber / result publisher topics. Robot starts sending data and processing results using this topics.

![Principle of 5G-ERA NetApp](Images/ML_service_principles.gif "ML service principles")

## Let's start

First of all, we will get the newest version of source files for this workshop, build the ROS2 packages and ensure, that we will have built packages available in all terminal windows by updating `.bashrc` one last time.

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

The commands above assume that the NetApp-Workshop repository is in the home directory.

## Basic example
In this example, we will use three parts of `ros2_5g_era_basic_example` package (`ml_service`, `image_publisher`, `results_listener`). 

![Basic Example Scheme](Images/Scheme_Basic_example.png "Basic Example Scheme")
This figure represents structure of this example.

First, we start the `ml_service` with the `DummyDetector` worker thread inside. At the start, `ml_service` will generate names for input and output topics that will be used by `image_publisher` and `results_listener`.

```bash
# Terminal 1
ros2 run ros2_5g_era_basic_example ml_service

# Terminal 2
ros2 topic list
```

In another terminal, we start the `ResultListener` node, which is originally subscribed to the `results` topic. We use remap function and change the topic to one of the results topics generated by the `ml_service` node.

```bash
# Terminal 3
ros2 run ros2_5g_era_basic_example result_listener --ros-args --remap results:=/robot1_results
```

Lastly, we run `ImagePublisher` Node, which reads the video file, converts each video frame to ROS2 `Image` message, and adds current `timestamp` and `frame_id` into the message's header. The `ImagaPublisher` node is publishing results to its `images` topic.

```bash
# Terminal 4
ros2 run ros2_5g_era_basic_example image_publisher --ros-args --remap images:=/robot1_images
```

## Standalone ML service - Basics

Before the start of this part, the communication interface between robot and ML services has to be defined - [5G-ERA ML Control Service Interface](CSS_Interface.md). 
This communication interface needs to be implemented by the robot in order to use any of the ML services.

Let's start our *Object Detection ML NetApp* in a standalone variant. For the purpose of this workshop, this service is implemented with a simple Face Detector from OpenCV library. The output of this object detection service is the `bounding box [x,y,w,h]` around people's faces. For completness, this service also returns `object class` and `detection score`.

![Standalone (basics) scheme](Images/Scheme_Standalone_basics.png "Standalone (basics) Scheme")
This figure represents structure of this example.

In this example, we will use `ml_service` part of `ros2_5g_era_object_detection_standalone_py` package, `image_publisher`, `results_listener` from previous example and definition of `ros2_5g_era_service_interfaces` for service call.

```bash
# Terminal 1
ros2 run ros2_5g_era_object_detection_standalone_py ml_service
```

After it is started, the `ml_service` waits for clients (robots) to connect. We can see, that ROS2 services provided by the `ml_control_services` node are now available.

```bash
ros2 service list
```

OUTPUT:
```
/ml_control_services/describe_parameters
/ml_control_services/get_parameter_types
/ml_control_services/get_parameters
/ml_control_services/list_parameters
/ml_control_services/set_parameters
/ml_control_services/set_parameters_atomically
/ml_control_services/start
/ml_control_services/state/get
/ml_control_services/state/reset
/ml_control_services/state/set
/ml_control_services/stop
```

We will simulate robot behaviour and complete ROS2 service call to `/ml_control_services/start` service using `ros2_5g_era_service_interfaces/Start` interfaces.

```bash
# Terminal 2 - Service call - Start
ros2 service call /ml_control_services/start ros2_5g_era_service_interfaces/Start 
```


**Control Service Server** inside running instance of `ml_service` there will be generated a unique identifier for this task (UUID) and launched a new ROS2 node with dedicated `ImageSubscriber` and `ResultPublisher` objects for this task, generating `/tasks/{id}/data` and `/tasks/{id}/results` topics, where `{id}` is the UUID.

```bash
ros@ros-VM:~$ ros2 service call /ml_control_services/start ros2_5g_era_service_interfaces/Start
requester: making request: ros2_5g_era_service_interfaces.srv.Start_Request(robot_worker_setting='')
response:
ros2_5g_era_service_interfaces.srv.Start_Response(success=True, message='', task_id='tb4a9f17340e143bc9cdfc2559dc503b3', data_topic='/tasks/tb4a9f17340e143bc9cdfc2559dc503b3/data', result_topic='/tasks/tb4a9f17340e143bc9cdfc2559dc503b3/results', worker_constraints='')
```


After that, we can start the `result_listener` and `image_publisher` from the `ros2_5g_era_basic_example` package as in the previous part of this workshop. This time, we will use the generated topics for remapping.

```bash
# Termial 3
ros2 run ros2_5g_era_basic_example result_listener --ros-args --remap results:=/tasks/{id}/results
# or
ros2 topic echo /tasks/{id}/results

# Terminal 4
ros2 run ros2_5g_era_basic_example image_publisher --ros-args --remap images:=/tasks/{id}/data

```

We can call `Stop service` with parameter `task_id: "{id}"` where `{id}` is the UUID generated for this task by ML Service, when `Start service` was called.
```bash
# Terminal 2 - Service call - Stop
ros2 service call /ml_control_services/stop ros2_5g_era_service_interfaces/Stop 'task_id: "{id}"'
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
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /ml_control_services}"

## Stop
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```
-->


## Standalone ML service in Kubernetes

Let's move on from the host system and deploy Standalone ML Service in Kubernetes. In this workshop, will we use `microk8s`, which is already prepared in **Workshop VM**. Update of configuration YAML files for Multus may be needed. We can check that using
[Kubernetes Deploy instructions](../NetApp_k8s_deploy/README.md).

### Simple Robot

For the purpose of this workshop, a simple robot in ROS2 was created. The robot implements **Client** part of **ML Control Service** for communication with ML Service. Despite that, it has its own ROS2 `robot_logic` node with`ros2_5g_era_robot_interfaces` for user commands.

`robot_logic/start_service` call will start **Control Service Client**, send the `Start` request to the server, and start internal image publisher and results subscriber. 

<!--#TODO: Nebude tam /robot_logic?-->

After that, the robot simply reads a video file and sends individual video frames to the `ml_service` for object detection.

`robot_logic/stop_service` will stop sending video data and send a `Stop` request to the server.


### Deploy and test

![Standalone (Kubernetes) scheme](Images/Scheme_Standalone_Kubernetes.png "Standalone (Kubernetes) scheme")
This figure represents structure of this example.

First of all, the ML Service has to be deployed in the Kubernetes cluster. Multus configuration was already checked.
In that case, we can apply `multus_config_vm.yaml` followed by `5gera_ml_service_standalone.yaml` and wait for their start.
```bash
# Terminal 1
cd ~/NetApp-Workshop/NetApp_k8s_deploy/ 

kubectl apply -f multus_config_vm.yaml # Possibly multus_config.yaml could work for host-system

kubectl apply -f 5gera_ml_service_standalone.yaml

watch "microk8s.kubectl get all"
```

Let's watch, what topics are available every 1 second.
```bash
# Terminal 2
watch -n 1 "ros2 topic list"
```

After ML service deployment in Kubernetes, we can start `robot_node` with default node name `robot_logic` and launch data processing using `robot_logic/start_service` call. Processing will stop after `robot_logic/stop_service` call.

```bash
# Terminal 3
ros2 run ros2_5g_era_robot_py robot_node

# Terminal 4
## Start 
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /ml_control_services}"

## Stop
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```

Another robot with a different name (e.g. `robot_logic_2`) can be started using a command line parameter.

```bash
# Terminal 5
ros2 run ros2_5g_era_robot_py robot_node -n robot_logic_2

# Terminal 4
## Start 
ros2 service call robot_logic_2/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /ml_control_services}"

## Stop
ros2 service call robot_logic_2/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```

Delete deployed ML service using the following command.

```bash
kubectl delete deployment.apps/ros-css-deployment
# TODO: Check deployment name
```

## Distributed ML service in Kubernetes

The Distributed Machine Learning service is composed of multiple parts, which are deployed in the Kubernetes cluster individually. The standalone version is broken down into four parts:

1. **ROS2 Control Service Server (ML interface)**
2. **Task broker** - System for distributing tasks to the running worker (Celery) and distribution channel (RabbitMQ). 
3. **Workers** - One or more instances of machine learning service (object detection) serving tasks.
4. **Result storage** - Celery backend for storing results of finished tasks. (Redis)

![Distributed (Kubernetes) scheme](Images/ML_Distributed_highlevel.png "Distributed ML service scheme (high level).")
This figure shows high-level concepts and different types of communication used for *distributed ML service*.

### Deploy and test
![Distributed (Kubernetes) scheme](Images/Scheme_Distributed_Kubernetes.png "Distributed ML service scheme.")
This figure represents structure of this example.

All parts of distributed ML service and their components are defined in `5gera_ml_service_distributed.yaml`. There are two environment variables, which has to be set properly:

```yaml
env:
- name: CELERY_BROKER_URL
    value: "amqp://guest:guest@localhost:5672"
- name: CELERY_RESULT_BACKEND
    value: "redis://localhost/"
```
Each of the four parts described previously may be deployed into different Kubernetes clusters. Only two conditions have to be met.

1. ML interface has to be visible for robots, wanting to process data.
2. Correct external IP addresses for RabbitMQ and Redis has to be set everywhere.


![Kubernetes cluest visualisation](Images/K8S_cluster_distributed_variant.png "Kubernetes cluest visualisation")
This figure shows visualisation of Kubernetes cluster and connections between pods and services. This structure will be deployed using configuration YAML file.

We will deploy the distributed variant inside a single Kubernetes cluster running on one computer. Both required URLs are set to `localhost` in the deployment YAML file.



```bash
# Terminal 1
cd ~/NetApp-Workshop/NetApp_k8s_deploy/


kubectl apply -f 5gera_ml_service_distributed.yaml

watch "microk8s.kubectl get all"
```

### OUTPUT

```bash
NAME                                              READY   STATUS    RESTARTS      AGE
pod/rabbit-deployment-644754765-m9s8j             1/1     Running   3 (61s ago)   19h
pod/ml-worker-deployment-6f89c9cbdc-n7v88         1/1     Running   3 (61s ago)   19h
pod/redis-deployment-6d94d9bb58-57ksg             1/1     Running   3 (61s ago)   19h
pod/distributed-css-deployment-7bb85d59bd-9vj4l   1/1     Running   3 (61s ago)   19h

NAME                       TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)    AGE
service/kubernetes         ClusterIP   10.152.183.1     <none>        443/TCP    46d
service/rabbitmq-service   ClusterIP   10.152.183.213   <none>        5672/TCP   19h
service/redis-service      ClusterIP   10.152.183.158   <none>        6379/TCP   19h

NAME                                         READY   UP-TO-DATE   AVAILABLE   AGE
deployment.apps/rabbit-deployment            1/1     1            1           19h
deployment.apps/ml-worker-deployment         1/1     1            1           19h
deployment.apps/redis-deployment             1/1     1            1           19h
deployment.apps/distributed-css-deployment   1/1     1            1           19h

NAME                                                    DESIRED   CURRENT   READY   AGE
replicaset.apps/rabbit-deployment-644754765             1         1         1       19h
replicaset.apps/ml-worker-deployment-6f89c9cbdc         2         2         2       19h
replicaset.apps/redis-deployment-6d94d9bb58             1         1         1       19h
replicaset.apps/distributed-css-deployment-7bb85d59bd   1         1         1       19h
```

Let's watch, what topics are available every 1 second.
```bash
# Terminal 2
watch "ros2 topic list"
```

After ML service deployment in Kubernetes, we can start `robot_node` with default node name `robot_logic` and launch data processing using `robot_logic/start_service` call. Processing will stop after `robot_logic/stop_service` call. 
```bash
# Terminal 3
ros2 run ros2_5g_era_robot_py robot_node

# Terminal 4
## Start 
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: /ml_control_services}"

## Stop
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```

Delete all deployments and services

```bash
kubectl delete --all deployments
kubectl delete --all services
```


<!--
```bash
kubectl delete deployment.apps/ml-worker-deployment
kubectl delete deployment.apps/distributed-css-deployment
kubectl delete deployment.apps/redis-deployment
kubectl delete deployment.apps/rabbit-deployment
kubectl delete service/rabbitmq-service
kubectl delete service/redis-service
```
-->

