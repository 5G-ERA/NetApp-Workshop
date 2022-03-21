# Prerequisities

This file describes prerequisities which has to be fulfilled by each workhop attendent before the 5G-ERA Workshop 3 - "Reference NetApp".
Please be sure, that all steps were finished before the workshop.
Online version of this file is available [here](https://github.com/5G-ERA/NetApp-Workshop/blob/81991375c1d8f407822297841d3c572c1a73b1c2/Documentation/Prerequisites.md).

## Workshop environment ready

This workshop builds on environment used in 5G-ERA Workshop 1 - "5G-ERA Middleware". Workshop attendents have to use this environment. In case, that your environment is not ready, you may download prepared virtual machine or configure your host system following configuration instructions. Both were created by University of Bedfordshire for 5G-ERA Workshop 1.

* [Workshop VM](https://universityofbedfordshire-my.sharepoint.com/:u:/g/personal/bartosz_bratus_study_beds_ac_uk/ETj6QI5cNN9Bv11wi6bRL4sBrF89RMamMbJ3pfcN8i2W6w?e=zAwf6c) (Size: 5GB. Hosted by University of Bedfordshire - Sharepoint)
* [Workshop VM - Mirror](https://nextcloud.fit.vutbr.cz/s/8HycYLqawximqrf) (Size: 5GB. Hosted by Brno Univesity of Technology - Nextcloud) - Use this link if you face difficulties when downloading from the original source.
* [VM Configuration](VM_configuration.md)

### VirtualBox host network
For freshly installed VirtualBox it is necessary to create VirtualBox Host Network to get the downloaded image running.
It can be simply added in VirtualBox GUI `File --> Host Network Manager... --> Create`. After this, created VirtualBox Host Network will be connected to the imported image.


## Further steps
To be able to follow this workshop, few further steps have to be completed to get the environment ready for use.

### Install ROS2 Galactic
This guide follows official ROS2 Galactic installation guide - https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html

<!--
#### Setup sources
```bash
# Enable Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 repository to system
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
-->

#### Setup sources
```bash
# Add ROS2 repository to system
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Install ROS2 packages
```bash
sudo apt update
sudo apt install -y ros-galactic-ros-base ros-galactic-demo-nodes*
sudo apt install -y python3-rosdep g++ python3-colcon-common-extensions tmux
sudo rosdep init
rosdep update
```

#### Updating .bashrc
Add sourcing of ROS2 Galactic and define kubectl alias for every open terminal window.

```bash
echo "# 5G-ERA Workshop - Reference NetApp" >> ~/.bashrc
echo "## Kubectl alias" >> ~/.bashrc
echo "alias kubectl=microk8s.kubectl" >> ~/.bashrc
echo "## Source ROS2 Galactic" >> ~/.bashrc
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Validate ROS2 installation
Run simple ROS2 talker-listener demo in two terminal windows.
```bash
# One terminal
ros2 run demo_nodes_cpp talker

# Second terminal
ros2 run demo_nodes_py listener
```

### Prepare workshop files
Clone source files for this workshop from [5G-ERA GitHub](https://github.com/5G-ERA/NetApp-Workshop)
```bash
git clone https://github.com/5G-ERA/NetApp-Workshop.git
cd ~/NetApp-Workshop/NetApp_ros2_src/
bash install_dependencies.sh
source set_environment.sh
```
#### Optional - Configure tmux
Presenter of this workshop will use `Terminal Multiplexer` with additional configuration for better experience. We will use the possibility to split window horizontally / vertically and supplement multiple terminal windows with one instance only.

`bind-key` is kept in default setting to `Ctrl+b`. Bind key has to be pressed firstly, then released and followed by another defined key.

Basic commands in updated config:
* Split window vertically: `bind-key,  |` 
* Split window horizontally: `bind-key, -`
* Move cursor between window panes:  `Alt + arrows`
* New terminal window: `Ctrl + DOWN (arrow)`
* Switch between windows: `Ctrl + LEFT/RIGHT (arrow)`

Copy setting and close terminal.
```bash
cp ~/NetApp-Workshop/.tmux.conf ~/.tmux.conf
```
Run `tmux` in new terminal window.


