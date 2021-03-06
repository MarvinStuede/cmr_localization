#!/bin/bash
#Script to install dependencies and RTAB-Map
rosdistro=$ROS_DISTRO
source /opt/ros/$rosdistro/setup.bash

#Add L-CAS repositories (topological navigation)
curl -s http://lcas.lincoln.ac.uk/repos/public.key | sudo apt-key add -
sudo apt-add-repository http://lcas.lincoln.ac.uk/ubuntu/main
sudo apt-get update

#Install and uninstall binaries for dependencies
sudo apt-get install -y ros-$rosdistro-rtabmap ros-$rosdistro-rtabmap-ros
sudo apt-get remove -y ros-$rosdistro-rtabmap ros-$rosdistro-rtabmap-ros

#Install GTSAM
git clone --branch 4.0.0-alpha2 https://bitbucket.org/gtborg/gtsam.git ~/opt/gtsam
cd ~/opt/gtsam
mkdir build
cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=OFF ..
make -j$((`nproc`-1))
sudo make install

#Install g2o
git clone https://github.com/RainerKuemmerle/g2o.git ~/opt/g2o
cd ~/opt/g2o
git checkout e9ccad6beaf3e3ebf77f474cf701ff164203b575
mkdir build
cd build
cmake ..
make -j$((`nproc`-1))
sudo make install

#Install libnabo
git clone https://github.com/ethz-asl/libnabo.git ~/opt/libnabo
cd ~/opt/libnabo
mkdir build
cd build
cmake ..
make -j$((`nproc`-1))
sudo make install

#Install libpointmatcher
git clone https://github.com/ethz-asl/libpointmatcher.git ~/opt/libpointmatcher
cd ~/opt/libpointmatcher
mkdir build
cd build
cmake ..
make -j$((`nproc`-1))
sudo make install


#Install RTAB Map
cd ~/opt
git clone https://github.com/introlab/rtabmap.git rtabmap
cd rtabmap/build
#Use commit from 29.07.2020
git checkout 6741355842c3a7ae5c84c8af6dcd21449644e88b
cmake ..
make -j$((`nproc`-1))
sudo make install
