
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <string.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>

#include <sstream>
class Talker
{
  public:
  Talker(std::string initial_msg)
    : m_msg(initial_msg)
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

    
    l_sub = m_nh.subscribe("/hsrb/base_scan", 5, &Talker::lasermessageCallback, this);
    p_sub = m_nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 5, &Talker::pointcloudCallback, this);

    
     
    m_pub= m_nh.advertise<sensor_msgs::LaserScan>("laser_output", 5);
  }
  private:

  ros::NodeHandle m_nh;
  boost::shared_ptr<tf2_ros::Buffer> tf2_;
  boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  int num_range=-1;
 

  std::string m_msg;
  ros::Subscriber l_sub;
  ros::Subscriber p_sub;
  ros::Publisher m_pub;
  
  

  



  sensor_msgs::LaserScan projected_laser;

  int pc_count=0;
  int num_points=0;
  
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    ros::Time begin_time = ros::Time::now();
    int num_points=0;

      // Container for original & filtered data
    pcl::PCLPointCloud2* cloudf = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloudf);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloudf);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01, 0.01, 0.01);
    sor.filter (cloud_filtered);

    // Convert to ROS data type
    //sensor_msgs::PointCloud2Ptr output;
    sensor_msgs::PointCloud2Ptr output = boost::make_shared<sensor_msgs::PointCloud2>(*cloud_msg);
    pcl_conversions::fromPCL(cloud_filtered, *output);

    if (num_range<0)
    {

      return;
    }

    ROS_INFO("pc callback");
  // Solve all of perception here...
    

    double min_height_= 0.0;
    double max_height_= 1.0;

    bool use_inf= true;
    double inf_epsilon= 1.0;
    double tolerance_=0.01;

     // determine amount of rays to create
    uint32_t ranges_size = std::ceil((projected_laser.angle_max - projected_laser.angle_min) / projected_laser.angle_increment);
    sensor_msgs::PointCloud2ConstPtr cloud_out;
    sensor_msgs::PointCloud2Ptr cloud;
    
    // Transform cloud if necessary
    if (!(projected_laser.header.frame_id == output->header.frame_id))
    {
      try
      {
        cloud.reset(new sensor_msgs::PointCloud2);
        tf2_->transform(*output, *cloud, projected_laser.header.frame_id, ros::Duration(tolerance_));
        cloud_out = cloud;
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform failure: " << ex.what());
        return;
      }
    }
    else
    {
      cloud_out = output;
    }

    projected_laser.ranges.assign(num_range,projected_laser.range_max);
    ROS_INFO("projected_msg size: %lu", projected_laser.ranges.size());

    // Iterate through pointcloud
    int num_points_total = cloud_out->height * cloud_out->width;
    int count_accepted=0;
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
        iter_z(*cloud_out, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
      {
        ROS_DEBUG_STREAM("rejected for nan in point(%f, %f, %f)\n"<< *iter_x<< *iter_y<< *iter_z);
        continue;
      }

      if (*iter_z > max_height_ || *iter_z < min_height_)
      {
        ROS_DEBUG_STREAM("rejected for height %f not in range (%f, %f)\n"<<*iter_z<< min_height_<< max_height_);
        continue;
      }

      double range = hypot(*iter_x, *iter_y);
      if (range < projected_laser.range_min)
      {
        ROS_DEBUG_STREAM("rejected for range %f below minimum value %f. Point: (%f, %f, %f)"<< range<< projected_laser.range_min<< *iter_x<<
                      *iter_y<< *iter_z);
        continue;
      }
      if (range > projected_laser.range_max)
      {
        ROS_DEBUG_STREAM("rejected for range %f above maximum value %f. Point: (%f, %f, %f)"<< range<<projected_laser.range_max<< *iter_x<<
                      *iter_y<< *iter_z);
        continue;
      }

      double angle = atan2(*iter_y, *iter_x);
      if (angle < projected_laser.angle_min || angle > projected_laser.angle_max)
      {
        ROS_DEBUG_STREAM("rejected for angle %f not in range (%f, %f)\n"<<angle<< projected_laser.angle_min<< projected_laser.angle_max);
        continue;
      }
      

    //   overwrite range at laserscan ray if new range is smaller
      int index = (angle - projected_laser.angle_min) / projected_laser.angle_increment;
      if (range < projected_laser.ranges[index])
      {
        projected_laser.ranges[index] = range;
        count_accepted++;
      
      }
      num_points=num_points+1;


    }
    ros::Duration end_time = ros::Time::now() - begin_time;
    ROS_INFO("Iteration in the PCL duration: %f", end_time.toSec());
    ROS_INFO("Number of points in the callback: %lu",num_points);







  }

  void lasermessageCallback(const sensor_msgs::LaserScanConstPtr& laser_msg)
  {
    ROS_INFO("Laser callback");
    sensor_msgs::LaserScan fused_laser;
    // temp.ranges=laser_msg->ranges;
    
    
    if (num_range<0)
    {
      ROS_INFO("setting up laser configurations");
      projected_laser.angle_min = laser_msg->angle_min;
      projected_laser.angle_max = laser_msg->angle_max;
      projected_laser.angle_increment=laser_msg->angle_increment;
      projected_laser.range_min=laser_msg->range_min;
      projected_laser.range_max=laser_msg->range_max;
      projected_laser.angle_increment=laser_msg->angle_increment;
      projected_laser.header.frame_id = laser_msg->header.frame_id;
      projected_laser.time_increment=laser_msg->time_increment;
      projected_laser.scan_time=laser_msg->scan_time;

      num_range=laser_msg->ranges.size();

    }

    if (laser_msg->ranges.size()!=projected_laser.ranges.size())
    {
      ROS_INFO("Laser msg size: %lu, projected_msg size: %lu", laser_msg->ranges.size(), projected_laser.ranges.size());
      m_pub.publish(*laser_msg);
      return;   

    }
    fused_laser.angle_min = laser_msg->angle_min;
    fused_laser.angle_max= laser_msg->angle_max;
    fused_laser.range_min=laser_msg->range_min;
    fused_laser.range_max=laser_msg->range_max;
    fused_laser.angle_increment=laser_msg->angle_increment;
    fused_laser.range_max=laser_msg->range_max;
    fused_laser.header=laser_msg->header;
    fused_laser.header.frame_id = laser_msg->header.frame_id;
    fused_laser.time_increment=laser_msg->time_increment;
    fused_laser.scan_time=laser_msg->scan_time;
    fused_laser.ranges.assign(num_range, 0);
    for (int i = 0; i < laser_msg->ranges.size(); i++)
    {
       fused_laser.ranges[i]= (laser_msg->ranges[i] < projected_laser.ranges[i])? laser_msg->ranges[i] : projected_laser.ranges[i];


    }

    projected_laser.header = laser_msg->header;
    m_pub.publish(fused_laser);   
    
  



  }

};

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  Talker t("s");
  ROS_INFO("initialized object");
  


   
  // ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {


    // t.talk_message(count);
    // ROS_INFO("while loop");
    ros::spinOnce();


  
  }
  return 0;
}


