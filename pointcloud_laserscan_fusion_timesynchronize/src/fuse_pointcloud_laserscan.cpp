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

#include <sstream>
class Talker
{
  public:
  Talker(std::string initial_msg)
    : m_msg(initial_msg)
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

    mCloudSub.subscribe(m_nh, "cloud_msg", 1);
    mLaserSub.subscribe(m_nh, "laser_msg", 1);
    mSync.reset(new Sync(MySyncPolicy(100), mCloudSub, mLaserSub));
    mSync->registerCallback(boost::bind(&Talker::filter_callback, this, _1, _2));

    //  l_sub = m_nh.subscribe("/hsrb/base_scan", 5, &Talker::lasermessageCallback, this);
    //  m_sub = m_nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 5, &Talker::messageCallback, this);

    
     
     m_pub= m_nh.advertise<sensor_msgs::LaserScan>("laser_output", 5);
  }
  private:

  ros::NodeHandle m_nh;
  boost::shared_ptr<tf2_ros::Buffer> tf2_;
  boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> mCloudSub;
  message_filters::Subscriber<sensor_msgs::LaserScan> mLaserSub;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::LaserScan> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> mSync;

  std::string m_msg;
  ros::Publisher m_pub;
  sensor_msgs::LaserScan output;


  void filter_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::LaserScanConstPtr& laser_msg )
  {
    ROS_INFO("Entering callback");
  // Solve all of perception here...
    output.angle_min=laser_msg->angle_min;
    output.angle_max= laser_msg->angle_max; 
    output.angle_increment= laser_msg->angle_increment; // M_PI/360.0
    output.scan_time= laser_msg->scan_time;
    output.time_increment=laser_msg->time_increment;
    output.range_min= laser_msg->range_min;
    output.range_max= laser_msg->range_max;
    output.ranges=laser_msg->ranges;

    double min_height_= 0.0;
    double max_height_= 1.0;
    std::string target_frame= "base_link"; 
  // output.header = cloud_msg->header;
  output.header=laser_msg->header;
  if (!target_frame.empty())
  {
    output.header.frame_id = target_frame;
  }

    
    bool use_inf= true;
    double inf_epsilon= 1.0;
    double tolerance_=0.01;

  
  // determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    // sensor_msgs::LaserScan output;

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    // if (use_inf)
    // {
    //   output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    // }
    // else
    // {
    //   output.ranges.assign(ranges_size, output.range_max + inf_epsilon);
    // }

      sensor_msgs::PointCloud2ConstPtr cloud_out;
      sensor_msgs::PointCloud2Ptr cloud;
    
    // Transform cloud if necessary
    if (!(output.header.frame_id == cloud_msg->header.frame_id))
    {
      try
      {
        cloud.reset(new sensor_msgs::PointCloud2);
        tf2_->transform(*cloud_msg, *cloud, target_frame, ros::Duration(tolerance_));
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
      cloud_out = cloud_msg;
    }

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
      if (range < output.range_min)
      {
        ROS_DEBUG_STREAM("rejected for range %f below minimum value %f. Point: (%f, %f, %f)"<< range<< output.range_min<< *iter_x<<
                      *iter_y<< *iter_z);
        continue;
      }
      if (range > output.range_max)
      {
        ROS_DEBUG_STREAM("rejected for range %f above maximum value %f. Point: (%f, %f, %f)"<< range<<output.range_max<< *iter_x<<
                      *iter_y<< *iter_z);
        continue;
      }

      double angle = atan2(*iter_y, *iter_x);
      if (angle < output.angle_min || angle > output.angle_max)
      {
        ROS_DEBUG_STREAM("rejected for angle %f not in range (%f, %f)\n"<<angle<< output.angle_min<< output.angle_max);
        continue;
      }

      // overwrite range at laserscan ray if new range is smaller
      int index = (angle - output.angle_min) / output.angle_increment;
      if (range < output.ranges[index])
      {
        output.ranges[index] = range;
        count_accepted++;
      }

  ROS_INFO_STREAM("accepted " << count_accepted << "/" << num_points_total << " points");
  m_pub.publish(output);   

    }

  }

//   void messageCallback(const std_msgs::String::ConstPtr& msg)
 


};

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  Talker t("s");
  


   
  // ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    // t.talk_message(count);
    ros::spinOnce();


    // loop_rate.sleep();

    ++count;
  }


  return 0;
}