# Prerequisities

This file describes prerequisities which has to be fulfilled by each workhop attendent before the 5G-ERA Workshop 3 - "Reference NetApp".
Please be sure, that all steps were finished before the workshop.

## Workshop environment ready

This workshop builds on environment used in 5G-ERA Workshop 1 - "5G-ERA Middleware". Workshop attendents have to use this environment. In case, that your environment is not ready, you may download prepared virtual machine or configure your host system following configuration instructions. Both were created by University of Bedfordshire for 5G-ERA Workshop 1

* [Workshop VM](https://universityofbedfordshire-my.sharepoint.com/:u:/g/personal/bartosz_bratus_study_beds_ac_uk/ETj6QI5cNN9Bv11wi6bRL4sBrF89RMamMbJ3pfcN8i2W6w?e=zAwf6c) 

* [VM Confiugration](VM_configuration.md)

## Further steps
To be able to follow this workshop, few further steps have to be completed to get the environment ready for use.

### Install ROS2 Galactic
This guide follows official ROS2 Galactic installation guide - https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html

#### Setup sources
```
# Enable Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 repository to system
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Install ROS2 packages
```
sudo apt update
sudo apt install ros-galactic-desktop


sudo apt install -y ros-galactic-ros-base ros-galactic-demo-nodes*
sudo apt install -y python3-rosdep g++ python3-colcon-common-extensions tmux
sudo rosdep init
rosdep update
```

#### Updating .bashrc
Add sourcing of ROS2 Galactic and define kubectl alias for every open terminal window.

```
echo "# 5G-ERA Workshop - Reference NetApp" >> ~/.bashrc
echo "## Source ROS2 Galactic" >> ~/.bashrc
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
echo "## Kubectl alias" >> ~/.bashrc
echo "alias kubectl=microk8s.kubectl" >> ~/.bashrc

source ~/.bashrc
```

#### Validate ROS2 installation
Run simple ROS2 talker-listener demo in two terminal windows.
```
# One terminal
ros2 run demo_nodes_cpp talker

# Second terminal
ros2 run demo_nodes_py listener
```

### Download workshop repository
```
git clone https://github.com/5G-ERA/NetApp-Workshop.git
```



