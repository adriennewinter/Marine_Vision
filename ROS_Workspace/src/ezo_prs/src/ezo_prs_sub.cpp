
#include <ros/ros.h>
#include <std_msgs/String.h>

void pressureCallback(const std_msgs::String::ConstPtr& msg)
{ 
  ROS_INFO("Pressure is: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ezo_prs_sub");
  ros::NodeHandle nh;
 
  ros::Subscriber pressure_sub = nh.subscribe("/ezo_prs_pressure", 1000, pressureCallback);

  ros::spin();

  return 0;
}
