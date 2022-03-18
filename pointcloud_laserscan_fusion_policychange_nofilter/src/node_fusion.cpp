

#include"Fusion.h"
int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "node_fusion");
  ros::NodeHandle nh;
  
  Fusion fusion(nh);
  ROS_INFO("initialized object");
  


   
  // ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {


    
    
    ros::spinOnce();


  
  }
  return 0;
}


