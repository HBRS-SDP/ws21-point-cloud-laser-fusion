
#include"Fusion.h"

int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "node_fusion");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;
  
  // Dedicated spinner for PCL
  ros::CallbackQueue callback_queue_pcl;
  nh2.setCallbackQueue(&callback_queue_pcl);

  Fusion fusion(nh, nh2);
  ROS_INFO("initialized object");
  

  std::thread spinner_thread_pcl([&callback_queue_pcl]() {
    ros::SingleThreadedSpinner spinner_pcl;
    spinner_pcl.spin(&callback_queue_pcl);
  });


  ros::spin();

  return 0;
}


