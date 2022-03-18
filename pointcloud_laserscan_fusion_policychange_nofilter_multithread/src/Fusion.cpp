
#include"Fusion.h"

Fusion::Fusion(ros::NodeHandle& nh, ros::NodeHandle& nh2):nh_(nh), nh2_(nh2)

{
    tf2_.reset(new tf2_ros::Buffer());
    tf2Listener_.reset(new tf2_ros::TransformListener(*tf2_));
    laserSub_ = nh_.subscribe("/hsrb/base_scan", 5, &Fusion::lasermessageCallback, this);
    pcSub_ = nh2_.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 5, &Fusion::pointcloudCallback, this);
    projectedLaserPub_= nh_.advertise<sensor_msgs::LaserScan>("laser_output", 5);
    pointCloudPub_=nh_.advertise<sensor_msgs::PointCloud2>("filtered_pcl",5);
}

void Fusion::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ros::Time begin_time = ros :: Time :: now();
    int numPoints_=0;
    

    if (numRange_<0)
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
    uint32_t ranges_size = std::ceil((projectedLaser_.angle_max - projectedLaser_.angle_min) / projectedLaser_.angle_increment);
    sensor_msgs::PointCloud2ConstPtr cloud_out;
    sensor_msgs::PointCloud2Ptr cloud;

    // Transform cloud if necessary
    if (!(projectedLaser_.header.frame_id == cloud_msg->header.frame_id))
    {
        try
        {
        cloud.reset(new sensor_msgs::PointCloud2);
        tf2_->transform(*cloud_msg, *cloud, projectedLaser_.header.frame_id, ros::Duration(tolerance_));
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

    projectedLaser_.ranges.assign(numRange_,projectedLaser_.range_max);
    ROS_INFO("projected_msg size: %lu", projectedLaser_.ranges.size());

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
            if (range < projectedLaser_.range_min)
            {
                ROS_DEBUG_STREAM("rejected for range %f below minimum value %f. Point: (%f, %f, %f)"<< range<< projectedLaser_.range_min<< *iter_x<<
                                *iter_y<< *iter_z);
                continue;
            }
            if (range > projectedLaser_.range_max)
            {
                ROS_DEBUG_STREAM("rejected for range %f above maximum value %f. Point: (%f, %f, %f)"<< range<<projectedLaser_.range_max<< *iter_x<<
                                *iter_y<< *iter_z);
                continue;
            }

            double angle = atan2(*iter_y, *iter_x);
            if (angle < projectedLaser_.angle_min || angle > projectedLaser_.angle_max)
            {
                ROS_DEBUG_STREAM("rejected for angle %f not in range (%f, %f)\n"<<angle<< projectedLaser_.angle_min<< projectedLaser_.angle_max);
                continue;
            }
            

            //overwrite range at laserscan ray if new range is smaller
            int index = (angle - projectedLaser_.angle_min) / projectedLaser_.angle_increment;
            if (range < projectedLaser_.ranges[index])
            {
                projectedLaser_.ranges[index] = range;
                count_accepted++;
            
            }
            numPoints_=numPoints_+1;

        }

    ros:: Duration end_time =ros::Time::now()-begin_time;
    ROS_INFO("Time for one iteration in the PCL Callback: %f", end_time.toSec());
    ROS_INFO("Number of points in the callback: %u",numPoints_);
}



void Fusion::lasermessageCallback(const sensor_msgs::LaserScanConstPtr& laser_msg)
{
    ROS_INFO("Laser callback");
    sensor_msgs::LaserScan fused_laser;
    // temp.ranges=laser_msg->ranges;
    if (numRange_<0)
    {
        ROS_INFO("setting up laser configurations");
        projectedLaser_.angle_min = laser_msg->angle_min;
        projectedLaser_.angle_max = laser_msg->angle_max;
        projectedLaser_.angle_increment=laser_msg->angle_increment;
        projectedLaser_.range_min=laser_msg->range_min;
        projectedLaser_.range_max=laser_msg->range_max;
        projectedLaser_.angle_increment=laser_msg->angle_increment;
        projectedLaser_.header.frame_id = laser_msg->header.frame_id;
        projectedLaser_.time_increment=laser_msg->time_increment;
        projectedLaser_.scan_time=laser_msg->scan_time;
        numRange_=laser_msg->ranges.size();

    }

    if (laser_msg->ranges.size()!=projectedLaser_.ranges.size())
    {
        ROS_INFO("Laser msg size: %lu, projected_msg size: %lu", laser_msg->ranges.size(), projectedLaser_.ranges.size());
        projectedLaserPub_.publish(*laser_msg);
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
    fused_laser.ranges.assign(numRange_, 0);
    for (int i = 0; i < laser_msg->ranges.size(); i++)
    {
        fused_laser.ranges[i]= (laser_msg->ranges[i] < projectedLaser_.ranges[i])? laser_msg->ranges[i] : projectedLaser_.ranges[i];


    }

    projectedLaser_.header = laser_msg->header;
    projectedLaserPub_.publish(fused_laser);   

}

