// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
// System: ROS1 Noetic Ubuntu 20
//
// This script opens multiple rosbags and saves select topics from each bag to a new combined bag.
// -----------------------------------------------------------------------------------------

#include <cstdio>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>

using namespace std;

string cam0_topic = "/video_source_0/raw"; 
string cam1_topic = "/video_source_1/raw";
string imu_topic = "/imu/data";
string prs_topic = "/pressure";
string folder_path = "/mnt/c/Users/adiee/OneDrive - University of Cape Town/Documents/MASTERS/Experimental Work/Dataset Creation/Ship-Hull-Vinyl-Dataset/Data/"; //  /full/path/to/folder/
string bag1_name = "ship_hull_vinyl_enhanced.bag";
string bag2_name = "ship_hull_vinyl_raw.bag";
string joined_bag_name = "enhanced_joined.bag"; // bag_name.bag (bag to be created)

int j, k, l, m = 0;

int main(int argc, char* argv[]){	
  // Create ROS node
  ros::init(argc, argv, "rosbag_join");
  ros::NodeHandle nh;
  
  // Open the bags
  rosbag::Bag joined_bag, split_bag_1, split_bag_2;
  joined_bag.open(folder_path+joined_bag_name, rosbag::bagmode::Write);
  split_bag_1.open(folder_path+bag1_name, rosbag::bagmode::Read);
  split_bag_2.open(folder_path+bag2_name, rosbag::bagmode::Read);

  // Create topics vector to iterate through
  vector<string> cam_topics, other_topics; 
  cam_topics.push_back(cam0_topic);
  cam_topics.push_back(cam1_topic);
  other_topics.push_back(imu_topic);
  other_topics.push_back(prs_topic);

  // Iterate through all messages on select topics in bag1 
  cout << "bag 1 writing cam topics." << endl;
  rosbag::View rosbagView_1(split_bag_1, rosbag::TopicQuery(cam_topics));
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView_1)
  {
    sensor_msgs::Image::ConstPtr img = msg.instantiate<sensor_msgs::Image>();
    if(msg.getTopic() == cam0_topic){
      if(img->header.stamp < ros::TIME_MIN){cout << "time min" << endl;}
        //joined_bag.write(cam0_topic, img->header.stamp, *img);
        k++;
    }
    else if(msg.getTopic() == cam1_topic){
      if(img->header.stamp < ros::TIME_MIN){cout << "time min" << endl;}
        //joined_bag.write(cam1_topic, img->header.stamp, *img);
        m++;
    }
  }
  cout << "bag 1 wrote " << k << " cam0 msgs." << endl;
  cout << "bag 1 wrote " << m << " cam1 msgs." << endl;
  
  // Iterate through all messages on select topics in bag2
  cout << "bag 2 writing imu and prs topics." << endl;
  rosbag::View rosbagView_2(split_bag_2, rosbag::TopicQuery(other_topics));
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView_2)
  {
    if(msg.getTopic() == imu_topic){
      sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
      joined_bag.write(imu_topic, imu->header.stamp, *imu);
      j++;
    }
    else if(msg.getTopic() == prs_topic){
      sensor_msgs::FluidPressure::ConstPtr prs = msg.instantiate<sensor_msgs::FluidPressure>();
      joined_bag.write(prs_topic, prs->header.stamp, *prs);
      l++;
    }
  }
  cout << "bag 2 wrote " << j << " imu msgs." << endl;
  cout << "bag 2 wrote " << l << " prs msgs." << endl;

    
  joined_bag.close();
  split_bag_1.close();
  split_bag_2.close();

  cout << "Process Complete." << endl;
  return 0;
}