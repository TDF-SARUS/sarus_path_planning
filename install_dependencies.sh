#!/bin/bash
#añadir clave de ros
sudo apt-key adv --fetch-keys 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' 
#añadir repositorio de ros
sudo apt-add-repository 'http://packages.ros.org/ros/ubuntu'                                  
#añadir clave de aerostack
wget -O - https://drive.upm.es/index.php/s/XLOZT2jqh77rvf7/download?path=public | sudo apt-key add -  
#añadir repositorio de aerostavk
sudo sh -c 'echo "deb https://drive.upm.es/index.php/s/XLOZT2jqh77rvf7/download?path=/dists/bionic ./" > /etc/apt/sources.list.d/aerostack-latest.list' 

#actualizar repositorios
sudo apt update
#instalar ros, git, y aerostack
sudo apt install ros-melodic-desktop-full -y
sudo apt install git -y
sudo apt install python3-argcomplete -y
sudo apt install aerostack-melodic -y

# Python
sudo apt install python3-pip
sudo -H pip3 install --upgrade pip
sudo pip3 install numpy geopy geopandas pandas tqdm shapely contextily matplotlib scikit-image rospkg

sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg

# Permisos
chmod +x planner.py
