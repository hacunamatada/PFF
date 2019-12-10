# PFF
Efficient Traversability Mapping Using a Point-cloud Fast Filter

The algorithm "PointCloud Fast Filter" (PFF) extract the relevant information from the Point-Cloud (RGB-D camera or 3D LiDAR), taking in account the height of the robot to build traversable mpas. The PFF is able to improve the accuracy and processing times compared to existing 2D and 3D solutions. We implemented PFF in C++ and integrated its operation in ROS Kinetic Kame in a laptop with Ubuntu 16.04.

## Subscribed Topics

### RGB-D Camera

* [/camera/depth/points] : Sensor PointCloud for peolple detection

### Velodyne

* [/velodyne_points] :  Sensor PointCloud for peolple detection



## Published Topics

* [/scan] :  LaserScan to be used with any 2D SLAM algorithm.


## Parameters

* [resolution] :  Angle resolution in degrees for filter the points in a cloud of size 360°/resolution.
* [sensor_height] :  S<sub>f</sub>, see the following image. in meters.
* [robot_height] :  S<sub>r</sub>, see the following image. in meters.

![alt text](https://drive.google.com/uc?export=view&id=16ya1gxClsY0DkhNTmTlV1ucT5h4f_jyX)

### Special parameters for Velodyne

* [horizontal_fov] :  Desired horizontal field of view (from -theta/2 to theta/2), see the following image. in degrees.

![alt text](https://drive.google.com/uc?export=view&id=1489zOF8vgnzcyRe783N9ieJLoMB6DqZj)


## How to build with catkin
```sh
$ cd ~/catkin_ws/src/
$ git clone https://github.com/cafemesa/PFF.git
$ cd ~/catkin_ws && catkin_make
```

## How to run

### Launch Files (roslaunch)

#### Requirements

To launch the mapping with the Astra Camera (from: https://github.com/orbbec/ros_astra_camera):

```sh
$ sudo apt install ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros
$ cd ~/catkin_ws/src
$ git clone https://github.com/orbbec/ros_astra_camera
$ ./ros_astra_camera/scripts/create_udev_rules
$ https://github.com/orbbec/ros_astra_launch.git
$ cd ~/catkin_ws && catkin_make
```

To launch the mapping with the Velodyne:

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/velodyne.git
$ cd ~/catkin_ws
$ rosdep install --from-paths src -i -y
$ catkin_make
```

#### Traversable mapping with Astra Camera PointCloud

```sh
$ roslaunch pff astra_pff.launch 
```

The previous command run:
* roslaunch astra_launch astrapro.launch
* rosrun pff astra_pff
* rivz with the configuration

#### Detection with Velodyne VLP-16 PointCloud

```sh
$ roslaunch pff velodyne_pff.launch 
```

The previous command run:
* roslaunch velodyne_pointcloud VLP16_points.launch
* rosrun pff velodyne_pff
* rivz with the configuration

### Nodes (rosrun)

#### Detection with Astra Camera PointCloud

```sh
$ rosrun pff astra_pff
$ roslaunch astra_launch astrapro.launch
$ rosrun rviz rviz -d ~/catkin_ws/src/pff/rviz_cfg/pff.rviz
```

#### Detection with Velodyne VLP-16 PointCloud

With the sensor:

```sh
$ rosrun pff velodyne_pff
$ roslaunch velodyne_pointcloud VLP16_points.launch
$ rosrun rviz rviz -d ~/catkin_ws/src/pff/rviz_cfg/pff.rviz
```

## Cite this work

@inproceedings{medinasanchez2019,
  title={Efficient Traversability Mapping for Service Robots Using a Point-cloud Fast Filter},
  author={Medina S{\'a}nchez, Carlos and Zella, Matteo and Capitan, Jes{\'u}s and Marron, Pedro J},
  booktitle={2019 19th International Conference on Advanced Robotics (ICAR)},
  pages={590--595},
  year={2019},
  organization={IEEE}
}

