# ML_Toolboxes

ML Toolboxes with unified input/output interface for 5G-ERA project.

## Test robot_logic - robot_ml_control_services_client - ml_control_services

Já to pouštím na Windows, ale na Linuxu by to mělo být asi takto:

Instalace závislostí (ROS2 Galactic), překlad
```console
cd ML_Toolboxes
bash install_dependencies.sh
colcon build
```
První terminál (stejná složka), první robot (robot_logic):
```console
source install/setup.sh
ros2 run ros2_5g_era_robot_py robot_node
```
Druhý terminál (stejná složka), druhý robot (robot_logic_2):
```console
source install/setup.sh
ros2 run ros2_5g_era_robot_py robot_node -n robot_logic_2
```
Třetí terminál (stejná složka), control services:
```console
source install/setup.sh
ros2 run ros2_5g_era_service_py service_node
```
Čtvrtý terminál (stejná složka), povely robotům:
```console
source install/setup.sh
```
Vnější povel prvnímu robotovi (robot_logic - start_service) aby robot_ml_control_services_client poslal požadavek ml_control_services se základním názvem "ml_service" s žádostí o přidělení názvů topiců a spuštění služby (ml_service_start).
```console
ros2 service call robot_logic/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: ml_service}"
```
Vnější povel druhému robotovi (robot_logic_2 - start_service) aby robot_ml_control_services_client poslal požadavek ml_control_services se základním názvem "ml_service" s žádostí o přidělení názvů topiců a spuštění služby (ml_service_start).
```console
ros2 service call robot_logic_2/start_service ros2_5g_era_robot_interfaces/srv/StartService "{service_base_name: ml_service}"
```
Vnější povel prvnímu robotovi (robot_logic - stop_service) aby robot_ml_control_services_client poslal požadavek ml_control_services na zastavení služby (ml_service_stop).
```console
ros2 service call robot_logic/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```
Vnější povel druhému robotovi (robot_logic_2 - stop_service) aby robot_ml_control_services_client poslal požadavek ml_control_services na zastavení služby (ml_service_stop).
```console
ros2 service call robot_logic_2/stop_service ros2_5g_era_robot_interfaces/srv/StopService
```