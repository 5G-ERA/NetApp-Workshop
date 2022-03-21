# Test robot_logic - robot_ml_control_services_client - ml_control_services

Instalation (ROS2 Galactic), building
```console
cd ML_Toolboxes
bash install_dependencies.sh
colcon build
```
First terminal (same dir), fisrt robot (robot_logic):
```console
source install/setup.sh
ros2 run ros2_5g_era_robot_py robot_node
```
Second terminal (same dir), second robot (robot_logic_2):
```console
source install/setup.sh
ros2 run ros2_5g_era_robot_py robot_node -n robot_logic_2
```
Third terminal (same dir), ML Control Services:
```console
source install/setup.sh
ros2 run ros2_5g_era_service_py service_node
```
Fourth terminal (same dir), commands to robots:
```console
source install/setup.sh
```
External command to the first robot (robot_logic - start_service) that the robot_ml_control_services_client send a ml_control_services request with the base name "ml_control_services" with a request to assign topic names and start the service (ml_service_start).
```console
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: ml_control_services}"
```
External command to the second robot (robot_logic_2 - start_service) that the robot_ml_control_services_client send a ml_control_services request with the base name "ml_control_services" with a request to assign topic names and start the service (ml_service_start).
```console
ros2 service call robot_logic_2/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: ml_control_services}"
```
External command to the first robot (robot_logic - stop_service) that robot_ml_control_services_client send a ml_control_services request to stop the service (ml_service_stop).
```console
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```
External command to the second robot (robot_logic_2 - stop_service) that robot_ml_control_services_client send a ml_control_services request to stop the service (ml_service_stop).
```console
ros2 service call robot_logic_2/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```