# ws21-point-cloud-laser-fusion
(WS2021)

# Pointcloud laser fusion

English 

[![v.1](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion/tree/ver_1_time_synchronization)]
[![v.2.0](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion/tree/ver_2.0_modified_policy_fusion)]
[![v.3](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion/tree/ver_3_added_filters_for_pointcloud)]
[![v.4](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion/tree/ver_4_run_subcribers_in_parallel)]

### First version v.1

This First version of the code, is performing fusion using time synchronization.

### Second version v.2.0

Modified the policy for point cloud fusion. Two separate callbacks , one for laser message and the other for pointcloud. The projection of point cloud happens in the pointcloud call back and the fusion with the latest laser message is done in the laser callback.

### Third version v.3

Include Voxel Grid and Pass through filter to reduce the number of points in the Pointcloud.  

### Fourth version v.4

Includes two subscribers that could run in parallel. 

## Introduction

We created this repository to fuse 3D point cloud data to laser scan data. 

To perform this task, we referred to the implementation in [ROS](http://wiki.ros.org/pointcloud_to_laserscan) , where a 3D point cloud is projected into a 2D laserscan. 

## Features

-Downsampling the PointCloud using a VoxelGrid filter. A number of points in a voxel are approximated  with their centroid.

-Downsampling the PointCloud using a PassThrough Filter along a specified dimension. In this case “z”. This means that the filter curs off values that are outside a specific range.

## Installation

Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

Installing from [Github source](https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion) is recommended :

```bash
git clone (https://github.com/HBRS-SDP/ws21-point-cloud-laser-fusion
```
## Run the code
After to clone the repository go to your catkin workspace , open terminal and: 
```bash
catkin build
roslaunch <package_name> <launch_file.launch>
rviz
```
