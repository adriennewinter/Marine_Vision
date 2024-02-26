// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
// System: ROS1 Noetic Ubuntu 20
//
// This script opens multiple rosbags with two image topics and joins them into one bag.
// -----------------------------------------------------------------------------------------

#include <cstdio>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

using namespace std;

string cam0_topic = "/cam_fl/image_raw/compressed"; // cemetery
string cam1_topic = "/cam_fr/image_raw/compressed";
// string cam0_topic = "/slave1/image_raw/compressed"; // Bus
// string cam1_topic = "/slave2/image_raw/compressed";
// string cam0_topic = "/cam0_left/image_raw"; // ACFR
// string cam1_topic = "/cam1_right/image_raw";
string folder_path = "/mnt/f/AFRL_Underwater_Stereo_Inertial_Pressure/Cemetery/"; //  /full/path/to/folder/
string joined_bag_name = "Cemetery_stereo_enhanced.bag"; // bag_name.bag (bag to be created)

int main(int argc, char* argv[]){
  cout << "joining bag: " << folder_path << joined_bag_name << endl;
	
  // Create ROS node
  ros::init(argc, argv, "rosbag_join");
  ros::NodeHandle nh;
  
  // Open the bag
  rosbag::Bag joined_bag;
  joined_bag.open(folder_path+joined_bag_name, rosbag::bagmode::Write);
  std::vector<std::string> topics; 
  topics.push_back(cam0_topic);
  topics.push_back(cam1_topic);
  
  // Open the bags to join
  rosbag::Bag split_bag_1, split_bag_2, split_bag_3, split_bag_4, split_bag_5;
  split_bag_1.open(folder_path+"split_1_enhanced.bag", rosbag::bagmode::Read);
  split_bag_2.open(folder_path+"split_2_enhanced.bag", rosbag::bagmode::Read);
  split_bag_3.open(folder_path+"split_3_enhanced.bag", rosbag::bagmode::Read);
  // split_bag_4.open(folder_path+"split_4_enhanced.bag", rosbag::bagmode::Read);
  // split_bag_5.open(folder_path+"split_5_enhanced.bag", rosbag::bagmode::Read);
  
  int i, j, k, l, m = 0;

  cout << "bag 1 writing." << endl;
  rosbag::View rosbagView_1(split_bag_1, rosbag::TopicQuery(topics));
  // Iterate through all messages on all topics in the bag 
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView_1)
  {
	sensor_msgs::Image::ConstPtr img = msg.instantiate<sensor_msgs::Image>();
	if(msg.getTopic() == cam0_topic){
		joined_bag.write(cam0_topic, ros::Time::now(), *img);
		k++;
	}
	else if(msg.getTopic() == cam1_topic){
		joined_bag.write(cam1_topic, ros::Time::now(), *img);
	}
  }
  cout << "bag 1 wrote " << k << " msgs." << endl;
  
  cout << "bag 2 writing." << endl;
  rosbag::View rosbagView_2(split_bag_2, rosbag::TopicQuery(topics));
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView_2)
  {
	sensor_msgs::Image::ConstPtr img = msg.instantiate<sensor_msgs::Image>();
	if(msg.getTopic() == cam0_topic){
		joined_bag.write(cam0_topic, ros::Time::now(), *img);
		j++;
	}
	else if(msg.getTopic() == cam1_topic){
		joined_bag.write(cam1_topic, ros::Time::now(), *img);
	}
  }
  cout << "bag 2 wrote " << j << " msgs." << endl;
  
  cout << "bag 3 writing." << endl;
  rosbag::View rosbagView_3(split_bag_3, rosbag::TopicQuery(topics));
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView_3)
  {
	sensor_msgs::Image::ConstPtr img = msg.instantiate<sensor_msgs::Image>();
	if(msg.getTopic() == cam0_topic){
		joined_bag.write(cam0_topic, ros::Time::now(), *img);
		i++;
	}
	else if(msg.getTopic() == cam1_topic){
		joined_bag.write(cam1_topic, ros::Time::now(), *img);
	}
  }
  cout << "bag 3 wrote " << i << " msgs." << endl;
  
  // cout << "bag 4 writing." << endl;
  // rosbag::View rosbagView_4(split_bag_4, rosbag::TopicQuery(topics));
  // BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView_4)
  // {
	// sensor_msgs::Image::ConstPtr img = msg.instantiate<sensor_msgs::Image>();
	// if(msg.getTopic() == cam0_topic){
	// 	joined_bag.write(cam0_topic, ros::Time::now(), *img);
	// 	l++;
	// }
	// else if(msg.getTopic() == cam1_topic){
	// 	joined_bag.write(cam1_topic, ros::Time::now(), *img);
	// }
  // }
  // cout << "bag 4 wrote " << l << " msgs." << endl;

  // cout << "bag 5 writing." << endl;
  // rosbag::View rosbagView_5(split_bag_5, rosbag::TopicQuery(topics));
  // BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView_5)
  // {
	// sensor_msgs::Image::ConstPtr img = msg.instantiate<sensor_msgs::Image>();
	// if(msg.getTopic() == cam0_topic){
	// 	joined_bag.write(cam0_topic, ros::Time::now(), *img);
	// 	m++;
	// }
	// else if(msg.getTopic() == cam1_topic){
	// 	joined_bag.write(cam1_topic, ros::Time::now(), *img);
	// }
  // }
  // cout << "bag 5 wrote " << m << " msgs." << endl;

  
  joined_bag.close();
  split_bag_1.close();
  split_bag_2.close();
  split_bag_3.close();
  // split_bag_4.close();
  // split_bag_5.close();

  cout << "Process Complete." << endl;
  
  return 0;
}