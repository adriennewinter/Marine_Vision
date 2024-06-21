// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
// System: ROS1 Melodic, Ubuntu 18.
//
// This script opens a rosbag file and saves synchronised IMU and Stereo Camera topics to a new rosbag, 
// while preserving the original frequency of the IMU compared to the cameras.
// The approximate time synchronizer filter is used from the ROS message_filters package together
// with additional buffers to achieve this.
//
// Edit the params.yaml file in the config folder before calling the launch file for the node.
// $ roslaunch synchronize stereo_inertial_synch.launch
// -----------------------------------------------------------------------------------------

#include <boost/foreach.hpp>
#include <deque>
#include <iostream>
#include <cstdio>
#include <cstddef>
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
#include <sensor_msgs/Image.h> // camera image messages
#include <sensor_msgs/Imu.h> // IMU messages

#include "synchronize/public_simple_filter.h"
#include "synchronize/synchronize.h"

using namespace std;

//-------------------------GLOBAL VARIABLES-----------------------------------------------------
std::string rosbag_folder_path;
std::string unsynched_bag_name;
std::string cam0_topic;
std::string cam1_topic;
std::string imu_topic;

struct stereo_inertial {
  sensor_msgs::Image img0;
  sensor_msgs::Image img1;
  sensor_msgs::Imu imu;
};

int i, k = 0; 
int l, m, n = 1;
std::deque<stereo_inertial> SynchedMsgsBuffer;
std::deque<sensor_msgs::Imu> imuBuffer;
std::vector<int> stamp_diffs_imu, stamp_diffs_img1;
std::map<int,ros::Time> all_stamps_imu, all_stamps_img1, all_stamps_img0;
std::vector<ros::Time> written_stamps_imu, written_stamps_img0, written_stamps_img1;

//-------------------------CALLBACKS-------------------------------------------------------
void SynchCallback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::Imu::ConstPtr& imu_synch_msg)
// Callback for synchronizing stereo messages with imu messages - higher IMU rate gets lost 
{ 
  struct stereo_inertial SynchedMsgsStruct;

  // Insert synched messages into the struct
  SynchedMsgsStruct.img0 = *img0_synch_msg;
  SynchedMsgsStruct.img1 = *img1_synch_msg;
  SynchedMsgsStruct.imu = *imu_synch_msg;

  // Insert the struct into the deque
  SynchedMsgsBuffer.push_back(SynchedMsgsStruct);

  // Find timestamp differences with respect to img0 and add to respective vectors
  int img1_diff = findStampDiffMsec(img0_synch_msg, img1_synch_msg);
  int imu_diff = findStampDiffMsec(img0_synch_msg, imu_synch_msg);
  stamp_diffs_img1.push_back(img1_diff);
  stamp_diffs_imu.push_back(imu_diff);
}


void imuBufferCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
// Add all IMU messages to a buffer (deque)
{
  imuBuffer.push_back(*imu_msg);
}

//-------------------------FUNCTIONS-------------------------------------------------------
void writeToBag(rosbag::Bag& synched_bag)
// Write synchronized messages to the synched rosbag
{
  sensor_msgs::Image img0_synch_msg, img1_synch_msg;
  sensor_msgs::Imu imu_synch_msg;
  struct stereo_inertial SynchedMsgsStruct;

  if(!SynchedMsgsBuffer.empty())
  {
    // Get latest synched message struct from the SynchedMsgsBuffer and delete it from the deque
    SynchedMsgsStruct = SynchedMsgsBuffer.front();
    img0_synch_msg = SynchedMsgsStruct.img0;
    img1_synch_msg = SynchedMsgsStruct.img1;
    imu_synch_msg = SynchedMsgsStruct.imu;
    SynchedMsgsBuffer.pop_front();
    
    // Look through the messages in the imuBuffer to find the same message that came through the synchronizer callback 
    for(int i=0; i<=imuBuffer.size(); i++)
    {     
      if(imu_synch_msg.header.stamp == imuBuffer.at(i).header.stamp)
      {
        // Write any earlier IMU messages that occured before the latest synched image-imu message to the rosbag
        while(imuBuffer.front().header.stamp != imu_synch_msg.header.stamp)
        { 
          synched_bag.write(imu_topic, imuBuffer.front().header.stamp, imuBuffer.front());
          written_stamps_imu.push_back(imuBuffer.front().header.stamp); // Store to find indices of dropped messages
          imuBuffer.pop_front();
          k++;
        }
        imuBuffer.pop_front(); // remove the synched IMU message from the buffer so we don't add it twice
        
        // Write the image-imu synched message to the rosbag
        synched_bag.write(imu_topic, img0_synch_msg.header.stamp, imu_synch_msg);
        synched_bag.write(cam0_topic, img0_synch_msg.header.stamp, img0_synch_msg); 
        synched_bag.write(cam1_topic, img0_synch_msg.header.stamp, img1_synch_msg);
        i++; k++;

        // Store to find indices of dropped messages
        written_stamps_imu.push_back(imu_synch_msg.header.stamp);
        written_stamps_img0.push_back(img0_synch_msg.header.stamp);
        written_stamps_img1.push_back(img1_synch_msg.header.stamp);

        break;
      }
    }
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
  topics.push_back(imu_topic);
  rosbag::View rosbagView(unsynched_bag, rosbag::TopicQuery(topics));
  cout << "Opening unsynched bag file." << endl;

  // Create empty rosbag to write synched messages into 
  rosbag::Bag synched_bag;
  synched_bag.open(rosbag_folder_path+"/"+"StereoInertialSynched.bag", rosbag::bagmode::Write); 

  // Set up public_simple_filters for message callbacks
  PublicSimpleFilter<sensor_msgs::Image> img0_filter;
  PublicSimpleFilter<sensor_msgs::Image> img1_filter;
  PublicSimpleFilter<sensor_msgs::Imu> imu_filter;
  
  // Create Approximate Time Synchronizer
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> approxTimePolicy;
  message_filters::Synchronizer<approxTimePolicy> sync(approxTimePolicy(100), img0_filter, img1_filter, imu_filter);
  sync.registerCallback(boost::bind(&SynchCallback, _1, _2, _3));

  // Register the IMU Buffer Callback
  imu_filter.registerCallback(imuBufferCallback);

  // Iterate through all messages on all topics in the bag and send them to their callbacks
  cout << "Writing to synched bag file. This may take a few minutes..." << endl;
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_filter.publicSignalMessage(img0); // call the SynchCallback
        all_stamps_img0[l] = img0->header.stamp; // record message index and timestamp 
        l++;
    }

    if (msg.getTopic() == cam1_topic)
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_filter.publicSignalMessage(img1); // call the SynchCallback
        all_stamps_img1[m] = img1->header.stamp; // record message index and timestamp 
        m++;
    }

    if (msg.getTopic() == imu_topic)
    {
      sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
      if (imu != NULL)
        imu_filter.publicSignalMessage(imu); // call the SynchCallback and imuBufferCallback
        all_stamps_imu[n] = imu->header.stamp; // record message index and timestamp
        n++;
    }

    writeToBag(synched_bag); // write to rosbag (disk) and empty the deques as callbacks are made to save RAM space
  }

  unsynched_bag.close();
  synched_bag.close();
  cout << "Closing both bag files." << endl;
}

//-------------------------MAIN-------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_inertial_synch");
  ros::NodeHandle nh;
  
  nh.getParam("rosbag_folder_path", rosbag_folder_path);
  nh.getParam("unsynched_bag_name", unsynched_bag_name);
  nh.getParam("cam0_topic", cam0_topic);
  nh.getParam("cam1_topic", cam1_topic);
  nh.getParam("imu_topic", imu_topic);
  
  synchronizeBag(rosbag_folder_path+"/"+unsynched_bag_name, nh);

  cout << "---" << endl;
  cout << "total messages written to bag:" << endl;
  cout << "cameras = " << i << endl;
  cout << "imu = " << k << endl;
  cout << "---" << endl;
  cout << "timestamp differences with respect to cam0:" << endl;
  cout << "max [cam1, imu] = " << *max_element(stamp_diffs_img1.begin(), stamp_diffs_img1.end()) << ", " << *max_element(stamp_diffs_imu.begin(), stamp_diffs_imu.end()) << ", " << "msecs" << endl;
  cout << "min [cam1, imu] = " << *min_element(stamp_diffs_img1.begin(), stamp_diffs_img1.end()) << ", " << *min_element(stamp_diffs_imu.begin(), stamp_diffs_imu.end()) << ", " << "msecs" << endl;
  cout << "average [cam1, imu] = " << round(accumulate(stamp_diffs_img1.begin(), stamp_diffs_img1.end(), 0.0)/stamp_diffs_img1.size()) << ", " << round(accumulate(stamp_diffs_imu.begin(), stamp_diffs_imu.end(), 0.0)/stamp_diffs_imu.size()) << ", " << "msecs" << endl;
  cout << "---" << endl;
  cout << "indices of dropped messages:" << endl;
  cout << "cam0 = " << endl;
  printDroppedInds(all_stamps_img0, written_stamps_img0);
  cout << "cam1 = " << endl;
  printDroppedInds(all_stamps_img1, written_stamps_img1);
  cout << "imu = " << endl;
  printDroppedInds(all_stamps_imu, written_stamps_imu); 
  cout << "---" << endl;
  cout << "Press Ctrl+C to kill the node." << endl;

  return 0;
}