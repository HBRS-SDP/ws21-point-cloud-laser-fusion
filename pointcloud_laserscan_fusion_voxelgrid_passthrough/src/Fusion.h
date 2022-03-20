//ROS specific includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//PCL Specific includes
// #include<pcl/ros/conversions.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>

#include <pcl/filters/passthrough.h>

//CPP specific includes
#include <string.h>
#include <sstream>



class Fusion
{
  public:

  Fusion(ros::NodeHandle& nh);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void lasermessageCallback(const sensor_msgs::LaserScanConstPtr& laser_msg);
  
  private:

  ros::NodeHandle nh_;
  boost::shared_ptr<tf2_ros::Buffer> tf2_;
  boost::shared_ptr<tf2_ros::TransformListener> tf2Listener_;
  int numRange_=-1;
  ros::Subscriber laserSub_;
  ros::Subscriber pcSub_;
  ros::Publisher projectedLaserPub_;
  ros::Publisher pointCloudPub_;
  sensor_msgs::LaserScan projectedLaser_;
  int numPoints_=0;
  int pcCount_=0;
};

