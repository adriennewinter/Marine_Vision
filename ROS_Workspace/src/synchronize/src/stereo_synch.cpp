// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
// System: ROS1 Melodic Ubuntu 18 and ROS1 Noetic Ubuntu 20
//
// This script opens a rosbag file and saves synchronised camera image topics to a new rosbag.
// It makes use of the ROS package message_filters - approximate time synchronizer.
// Written and tested in ROS1 Melodic environment.
//
// Edit the params.yaml file in the config folder before calling the launch file for the node.
// $ roslaunch synchronize stereo_synch.launch
// -----------------------------------------------------------------------------------------

#include <boost/foreach.hpp>
#include <deque>
#include <iostream>
#include <cstdio>
#include <string>
#include <cmath>
#include <numeric>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/simple_filter.h>
#include <sensor_msgs/Image.h>

#include "synchronize/public_simple_filter.h"
#include "synchronize/synchronize.h"

using namespace std;

//-------------------------GLOBAL VARIABLES-----------------------------------------------------
std::string rosbag_folder_path;
std::string unsynched_bag_name;
std::string cam0_topic;
std::string cam1_topic;

std::queue<sensor_msgs::Image> img0_queue, img1_queue;
std::vector<int> stamp_diffs_img1;
int i = 0; 

//-------------------------FUNCTIONS-------------------------------------------------------
void synchFilterCallback(const sensor_msgs::Image::ConstPtr& img0_msg, const sensor_msgs::Image::ConstPtr& img1_msg)
// Callback for synchronizing stereo messages (approximate time synchronizer - message_filter) and saving them in a queue
{ 
  img0_queue.push(*img0_msg);
  img1_queue.push(*img1_msg);

  // Find timestamp differences with respect to img0 and add to respective vectors
  int img1_diff = findStampDiffMsec(img0_msg, img1_msg);
  stamp_diffs_img1.push_back(img1_diff);
}



void writeToBag(rosbag::Bag& synched_bag)
// Write queued synchronized image messages to the synched rosbag
{
  sensor_msgs::Image img0_msg, img1_msg;

  if(!img0_queue.empty() && !img1_queue.empty())
  {
    // Get latest synched messages from queues and remove the messages from the queues
    img0_msg = img0_queue.front();
    img1_msg = img1_queue.front();
    img0_queue.pop();
    img1_queue.pop();

    // Write a synched pair of messages to a rosbag
    synched_bag.write(cam0_topic, img0_msg.header.stamp, img0_msg); 
    synched_bag.write(cam1_topic, img0_msg.header.stamp, img1_msg);
    i += 1;
  }
}



void synchronizeBag(const std::string& filename, ros::NodeHandle& nh)
// Load rosbag, iterate through the messages on each topic and call the synchronizer callback
{
  // Load unsynched rosbag
  rosbag::Bag unsynched_bag;
  unsynched_bag.open(filename, rosbag::bagmode::Read);
  std::vector<std::string> topics; // create a vector of topics to iterate through
  topics.push_back(cam0_topic);
  topics.push_back(cam1_topic);
  rosbag::View rosbagView(unsynched_bag, rosbag::TopicQuery(topics));
  cout << "Opening unsynched bag file." << endl;

  // Create empty rosbag to write synched messages into 
  rosbag::Bag synched_bag;
  synched_bag.open(rosbag_folder_path+"/"+"StereoSynched.bag", rosbag::bagmode::Write); 

  // Set up public_simple_filters for message callbacks
  PublicSimpleFilter<sensor_msgs::Image> img0_filter;
  PublicSimpleFilter<sensor_msgs::Image> img1_filter;
  
  // Use an approximate time synchronizer to synchronize image messages
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> approxTimePolicy;
  message_filters::Synchronizer<approxTimePolicy> sync(approxTimePolicy(100), img0_filter, img1_filter);
  sync.registerCallback(boost::bind(&synchFilterCallback, _1, _2));

  // Iterate through all messages on all topics in the bag and send them to the synchronizer callback
  cout << "Writing to synched bag file. This may take a few minutes..." << endl;
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_filter.publicSignalMessage(img0); // call the synchFilterCallback
    }

    if (msg.getTopic() == cam1_topic)
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_filter.publicSignalMessage(img1); 
    }
    writeToBag(synched_bag); // write to the rosbag (disk) and empty the image queues as callbacks are made to save RAM space
  }
  unsynched_bag.close();
  synched_bag.close();
  cout << "Closing both bag files." << endl;
}



//-------------------------MAIN-------------------------------------------------------
int main(int argc, char** argv)
{
  // Create synchronize ROS node
  ros::init(argc, argv, "stereo_synch");
  ros::NodeHandle nh;
  
  nh.getParam("rosbag_folder_path", rosbag_folder_path);
  nh.getParam("unsynched_bag_name", unsynched_bag_name);
  nh.getParam("cam0_topic", cam0_topic);
  nh.getParam("cam1_topic", cam1_topic);

  synchronizeBag(rosbag_folder_path+"/"+unsynched_bag_name, nh);

  cout << "Total synched camera messages written to bag = " << i << endl;
  cout << "---" << endl;
  cout << "cam1 timestamp differences with respect to cam0:" << endl;
  cout << "max = " << *max_element(stamp_diffs_img1.begin(), stamp_diffs_img1.end()) << " msecs" << endl;
  cout << "min = " << *min_element(stamp_diffs_img1.begin(), stamp_diffs_img1.end()) << " msecs" << endl;
  cout << "average = " << round(accumulate(stamp_diffs_img1.begin(), stamp_diffs_img1.end(), 0.0)/stamp_diffs_img1.size()) << " msecs" << endl;
  cout << "---" << endl;
  cout << "Press Ctrl+C to kill the node." << endl;
 
  return 0;
}