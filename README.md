# hector_slam_g2o

A ROS Package for Pose Graph SLAM that uses hector_slam for the front end and g2o for backend.


## Requirements
***hector_slam_g2o*** requires the following libraries:

- G2O
- PCL
- Ceres
- Eigen3


The following ROS packages are required:

- nav_msgs
- roscpp
- sensor_msgs
- tf
- tf_conversions
- laser_geometry
- fast_gicp
- geometry_msgs


## How to Use

```zsh
roslaunch hector_slam_g2o hector_slam_g2o.launch
```
