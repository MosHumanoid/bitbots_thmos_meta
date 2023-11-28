# bitbots_thmos_meta

> This repository is forked from Robocup team Bitbots and modified for lesson. 

## install

depends:
* ros with python3

**noetic**
```shell
sudo apt install python3-catkin-tools python3-pip ros-noetic-amcl ros-noetic-controller-interface ros-noetic-controller-manager ros-noetic-controller-manager-msgs ros-noetic-gazebo-ros-control ros-noetic-hector-gazebo ros-noetic-hector-gazebo-plugins ros-noetic-imu-sensor-controller ros-noetic-joint-state-controller ros-noetic-joint-trajectory-controller ros-noetic-map-server ros-noetic-move-base ros-noetic-moveit ros-noetic-moveit-core ros-noetic-moveit-resources ros-noetic-moveit-ros-planning ros-noetic-moveit-ros-planning-interface ros-noetic-plotjuggler ros-noetic-pointcloud-to-laserscan ros-noetic-robot-controllers ros-noetic-robot-controllers-interface ros-noetic-robot-controllers-msgs ros-noetic-robot-localization ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-rqt-controller-manager ros-noetic-rqt-joint-trajectory-controller ros-noetic-spatio-temporal-voxel-layer ros-noetic-rviz-imu-plugin ros-noetic-imu-complementary-filter ros-noetic-joy ros-noetic-ros-numpy
```

**melodic**
```shell
sudo apt install python3-catkin-tools python3-pip ros-melodic-amcl ros-melodic-controller-interface ros-melodic-controller-manager ros-melodic-controller-manager-msgs ros-melodic-gazebo-ros-control ros-melodic-hector-gazebo ros-melodic-hector-gazebo-plugins ros-melodic-imu-sensor-controller ros-melodic-joint-state-controller ros-melodic-joint-trajectory-controller ros-melodic-map-server ros-melodic-move-base ros-melodic-moveit ros-melodic-moveit-core ros-melodic-moveit-resources ros-melodic-moveit-ros-planning ros-melodic-moveit-ros-planning-interface ros-melodic-plotjuggler ros-melodic-pointcloud-to-laserscan ros-melodic-robot-controllers ros-melodic-robot-controllers-interface ros-melodic-robot-controllers-msgs ros-melodic-robot-localization ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-rqt-controller-manager ros-melodic-rqt-joint-trajectory-controller ros-melodic-spatio-temporal-voxel-layer ros-melodic-rviz-imu-plugin ros-melodic-imu-complementary-filter ros-melodic-joy ros-melodic-ros-numpy ros-melodic-yocs-velocity-smoother
```

**install rospkg**
```shell
sudo pip3 install rospkg
```

**change mode** 

```shell
chmod 777 bitbots_thmos_meta -R
find bitbots_thmos_meta -name "*.py" | xargs dos2unix
```

## usage

**motion test**

shell 1
```shell
roslaunch thmos_bringup motion_viz.launch
```

shell 2
```shell
rosrun thmos_bringup keyboard.py
```

**webots sim test**

shell 1
```shell
roslaunch thmos_bringup motion_noviz.launch
```

shell 2
```shell
webots thmos_webots_sim/worlds/thmos_webots_sim.wbt
```

shell 3
```shell
rosrun thmos_bringup keyboard.py
```
