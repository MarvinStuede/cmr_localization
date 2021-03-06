# **cmr_localization**: Starters and packages for mapping and localization in large-scale environments

This repository contains the packages related to mapping and localization with [RTAB-Map](http://introlab.github.io/rtabmap/) with individual configurations for Sobi as well as the Map Management package to automatically create maps for different environments.
For instructions to the specific functionalities, see the READMEs in the specific packages of this repository.
The README of cmr_localization is a good start point.

### Prerequisites and Installing
See general instructions [here](https://marvinstuede.github.io/Sobi/software/).
After you completed the described steps, you should remove RTAB-Map since we will install it from source
```
sudo apt-get remove ros-$ROS_DISTRO-rtabmap -y
```
Then install RTAB-Map with dependencies (This may take a while).
```
bash src/cmr_localization/cmr_localization/scripts/install_dependencies.sh
```
Then compile your workspace
```
catkin build --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON
```
