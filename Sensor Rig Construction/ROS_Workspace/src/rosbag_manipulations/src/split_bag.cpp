// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
// System: ROS1 Noetic Ubuntu 20
//
// This script opens a rosbag with two image topics and splits it into smaller bags.
// -----------------------------------------------------------------------------------------

#include <cstdio>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

using namespace std;

// Bus - 328secs - 4250 - 235 msgs = 4015 - 5 bags of 803 msgs
// Cemetery - 143secs - 1596 - 170 msgs = 1426 msgs - 2 bags of 713 msgs

string cam0_topic = "/cam_fl/image_raw/compressed";
string cam1_topic = "/cam_fr/image_raw/compressed";
string folder_path = "/mnt/f/AFRL_Underwater_Stereo_Inertial_Pressure/Cemetery/"; //  /full/path/to/folder/
string original_bag_name = "cemetery_loop_002_corrected.bag"; // bag_name.bag
int split = 713;
int initial_cut = 170;

int main(int argc, char* argv[]){
  cout << "splitting bag: " << folder_path << original_bag_name << endl;
	
  // Create ROS node
  ros::init(argc, argv, "rosbag_split");
  ros::NodeHandle nh;
  
  // Open the bag
  rosbag::Bag original_bag;
  original_bag.open(folder_path+original_bag_name, rosbag::bagmode::Read);
  std::vector<std::string> topics; 
  topics.push_back(cam0_topic);
  topics.push_back(cam1_topic);
  rosbag::View rosbagView(original_bag, rosbag::TopicQuery(topics));
  
  // Create new bags to split into
  rosbag::Bag split_bag_1, split_bag_2, split_bag_3, split_bag_4, split_bag_5;
  split_bag_1.open(folder_path+"split_1.bag", rosbag::bagmode::Write);
  split_bag_2.open(folder_path+"split_2.bag", rosbag::bagmode::Write);
  //split_bag_3.open(folder_path+"split_3.bag", rosbag::bagmode::Write);
  //split_bag_4.open(folder_path+"split_4.bag", rosbag::bagmode::Write);
  //split_bag_5.open(folder_path+"split_5.bag", rosbag::bagmode::Write);
  
  int i, j, k, l = 0;
  sensor_msgs::Image img0, img1;

  // Iterate through all messages on all topics in the bag 
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
	if(msg.getTopic() == cam0_topic){
		if(i >= initial_cut){ // skip some initial frames
			if(k < split){
				sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
				split_bag_1.write(cam0_topic, (*img0).header.stamp, *img0);
				k++;
				cout << "bag 1 write. k = " << k << endl;
			}
			else if(k < split*2){
				sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
				split_bag_2.write(cam0_topic, (*img0).header.stamp, *img0);
				k++;
				cout << "bag 2 write. k = " << k << endl;
			}
			// else if(k < split*3){
			// 	sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
			// 	split_bag_3.write(cam0_topic, (*img0).header.stamp, *img0);
			// 	k++;
			// 	cout << "bag 3 write. k = " << k << endl;
			// }
			// else if(k < split*4){
			// 	sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
			// 	split_bag_4.write(cam0_topic, (*img0).header.stamp, *img0);
			// 	k++;
			// 	cout << "bag 4 write. k = " << k << endl;
			// }
			// else if(k < split*5){
			// 	sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
			// 	split_bag_5.write(cam0_topic, (*img0).header.stamp, *img0);
			// 	k++;
			// 	cout << "bag 5 write. k = " << k << endl;
			// }
		}
		i++;
	}
	else if(msg.getTopic() == cam1_topic){
		if(j >= initial_cut){
			if(l < split){
				sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
				split_bag_1.write(cam1_topic, (*img1).header.stamp, *img1);
				l++;
			}
			else if(l < split*2){
				sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
				split_bag_2.write(cam1_topic, (*img1).header.stamp, *img1);
				l++;
			}
			// else if(l < split*3){
			// 	sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
			// 	split_bag_3.write(cam1_topic, (*img1).header.stamp, *img1);
			// 	l++;
			// }
			// else if(l < split*4){
			// 	sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
			// 	split_bag_4.write(cam1_topic, (*img1).header.stamp, *img1);
			// 	l++;
			// }
			// else if(l < split*5){
			// 	sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
			// 	split_bag_5.write(cam1_topic, (*img1).header.stamp, *img1);
			// 	l++;
			// }
		}			
		j++;
	}
  }// boost foreach
  
  original_bag.close();
  split_bag_1.close();
  split_bag_2.close();
  split_bag_3.close();
  split_bag_4.close();
  split_bag_5.close();
  
  return 0;
}