# **cmr_localization**: Starters for mapping and localization
This package contains different launch files, configs and maps to start localization and mapping of the robot.

### Mapping
To start mapping with RTAB-Map and the cmr_lidarloop extension you can run the launch file
```
roslaunch cmr_localization mapping.launch
```
This will, by default, create a new RTAB-Map database in `/.ros/rtabmap.db`.
To see which RTAB-parameters are used you can check the `.yaml` files in the `cfg` folder.
There are three config files:
 1. `rtabmap_common.yaml`
The file contains the parameters, which will always be used.
 2. `rtabmap_long_range.yaml`
These parameters are used for the *long_range* configuration. This configuration is used for large scale areas (e.g. outdoor or large rooms). Instead of the depth information from the RGBD-Cameras, this config uses a depth image created from the 3D-Lidar. To enable this configuration, change the `long_range` arg to true in `rtabmap.launch`.
 3. `rtabmap_short_range.yaml`
These parameters are used for the *short_range* configuration. This configuration is the standard configuration and can be used e.g. for indoor areas (office environments, hallways etc.).

### Localization
Localization is basically the same mapping, configuration wise, but RTAB-Map is started in localization mode (to not delete the db on start and not add nodes to the graph).
Start via
```
roslaunch cmr_localization localization.launch
```
