# Pointcloud Laser fusion

English 


[![v.1](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion/tree/ver_1_time_synchronization)]
[![v.2.0](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion/tree/ver_2.0_modified_policy_fusion)]
[![v.3](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion/tree/ver_3_added_filters_for_pointcloud)]

[![v.4](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion/tree/ver_4_run_subcribers_in_parallel)]

### First Version v.1

This First version of the code, is performing fusion using time synchronization.

### Second Version v.2.0

In this version the time synchronization policy was modified in order to deal the problem of freeze and lag of the first version v.1

### Third Version v.3

The third version include Voxel Grid and Pass trough filter to reduce the number of points of the PCL. 

### Fourth Version v.4

The fourth version implements MultiThreading.

## Introduction

A node for converts a 3D point cloud into a 2D laser scan is developed in [ROS](http://wiki.ros.org/pointcloud_to_laserscan) but it does not fuse both data. Therefore we created this repository to fuse the 3D point cloud data  into the 2D laser scan data. 

## Features



-Downsampling the PointCloud using a VoxelGrid filter. A number of points in a voxel are approximated  with their centroid.

-Downsampling the PointCloud using a PassThrough Filter along a specified dimension. In this case “z”. This means that the filter curs off values that are outside a specific range.


## Installation

Installing from [Github source](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion) is recommended :

```bash
git clone (https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion

```

## Run the code

After to clone the repository go to your catkin workspace , open terminal and: 

```bash
catkin build
roslaunch pointcloud_laserscan_fusion pointcloud_laser_fuse.launch 
rviz
```
