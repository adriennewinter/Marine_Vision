// -----------------------------------------------------------------------------------------
// Author: Adrienne Winter, 2023
// System: ROS1 Melodic, Ubuntu 18.
//
// This script opens a rosbag file and saves synchronised pressure sensor, IMU and Stereo Camera
// topics to a new rosbag, while preserving the original frequencies of each sensor.
// The Approximate Time Synchronizer filter is used from the ROS 'message_filters' package together
// with additional buffers to achieve synchronization.
//
// Note that some messages may be dropped due to sensor streams starting at different times at the
// beginning of the bag recording.
//
// Edit the params.yaml file in the config folder before calling the launch file for the node.
// $ roslaunch synchronize synchronize_node.launch
// -----------------------------------------------------------------------------------------

#include <boost/foreach.hpp>
#include <deque>
#include <iostream>
#include <cstdio>
#include <cstddef>
#include <string>
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
#include <sensor_msgs/FluidPressure.h> // pressure sensor messages

#include "synchronize/public_simple_filter.h"

using namespace std;

//-------------------------GLOBAL VARIABLES-----------------------------------------------------
std::string rosbag_folder_path;
std::string unsynched_bag_name;
std::string cam0_topic;
std::string cam1_topic;
std::string imu_topic;
std::string prs_topic;

struct synched_struct {
  sensor_msgs::Image img0;
  sensor_msgs::Image img1;
  sensor_msgs::Imu imu;
  sensor_msgs::FluidPressure prs;
};

int m, n, o = 0; 
std::deque<synched_struct> Synch1Buffer, Synch2Buffer; // a deque of structs
std::deque<sensor_msgs::Imu> imuBuffer;
std::deque<sensor_msgs::FluidPressure> prsBuffer;
std::vector<float> stamp_diffs_imu, stamp_diffs_img1, stamp_diffs_prs;

//-------------------------CALLBACKS------------------------------------------------------------
void Synch1Callback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::FluidPressure::ConstPtr& prs_synch_msg)
// Callback for synchronizing pressure sensor messages with stereo messages - higher camera rate gets lost 
{ 
  struct synched_struct SynchedMsgsStruct;

  // Insert synched messages into the struct 
  SynchedMsgsStruct.img0 = *img0_synch_msg;
  SynchedMsgsStruct.img1 = *img1_synch_msg;
  SynchedMsgsStruct.prs = *prs_synch_msg;

  // Insert the struct into the deque
  Synch1Buffer.push_back(SynchedMsgsStruct);

  // Find timestamp differences with respect to img0
  float img0_stamp = img0_synch_msg->header.stamp.toSec();
  float img1_stamp = img1_synch_msg->header.stamp.toSec();
  float prs_stamp = prs_synch_msg->header.stamp.toSec();
  float img1_diff = img0_stamp - img1_stamp;
  float prs_diff = img0_stamp - prs_stamp;

  // Add stamp diffs to respective vectors
  stamp_diffs_img1.push_back(img1_diff);
  stamp_diffs_prs.push_back(prs_diff);
}


void Synch2Callback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::Imu::ConstPtr& imu_synch_msg)
// Callback for synchronizing stereo messages with IMU messages - higher IMU rate gets lost 
{ 
  struct synched_struct SynchedMsgsStruct;

  // Insert synched messages into the struct
  SynchedMsgsStruct.img0 = *img0_synch_msg;
  SynchedMsgsStruct.img1 = *img1_synch_msg;
  SynchedMsgsStruct.imu = *imu_synch_msg;

  // Insert the struct into the deque
  Synch2Buffer.push_back(SynchedMsgsStruct);

  // Find timestamp differences with respect to img0
  float img0_stamp = img0_synch_msg->header.stamp.toSec();
  float img1_stamp = img1_synch_msg->header.stamp.toSec();
  float imu_stamp = imu_synch_msg->header.stamp.toSec();
  float img1_diff = img0_stamp - img1_stamp;
  float imu_diff = img0_stamp - imu_stamp;

  // Add stamp diffs to respective vectors
  stamp_diffs_img1.push_back(img1_diff);
  stamp_diffs_imu.push_back(imu_diff);

}


void imuBufferCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
// Add all IMU messages to a buffer (deque) so that we can preserve higher IMU rate
{
  imuBuffer.push_back(*imu_msg);
}


//-------------------------FUNCTIONS-------------------------------------------------------
void writeToBag(rosbag::Bag& synched_bag)
// Write synchronized messages to the synched rosbag
{
  sensor_msgs::Image img0_synch1_msg, img1_synch1_msg, img0_synch2_msg, img0_synch2_front, img1_synch2_front, img0_synch2_msg_plusOne;
  sensor_msgs::Imu imu_synch_msg_front, imu_synch2_msg;
  sensor_msgs::FluidPressure prs_synch1_msg;
  struct synched_struct Synch1Struct, Synch2Struct, Synch2Struct_plusOne;

  if(!Synch1Buffer.empty())
  {
    // Get latest synched message struct from the Synch1Buffer
    Synch1Struct = Synch1Buffer.front();
    img0_synch1_msg = Synch1Struct.img0;
    img1_synch1_msg = Synch1Struct.img1;
    prs_synch1_msg = Synch1Struct.prs;
    
    // Find the message in Synch2Buffer that equals the current Synch1Buffer message
    for(int i=0; i<Synch2Buffer.size(); i++) // loop 1
    {
      // Get the current img0 message from the Synch2Buffer for comparison with Synch1Buffer
      Synch2Struct = Synch2Buffer.at(i);
      img0_synch2_msg = Synch2Struct.img0;

      // If the current Synch2 message is equal to the current Synch1 message 
      if(img0_synch1_msg.header.stamp == img0_synch2_msg.header.stamp)
      {
        if(Synch2Buffer.size() > (i+2)) 
        {
          // Get the front message from Synch2Buffer 
          Synch2Struct = Synch2Buffer.front();
          img0_synch2_front = Synch2Struct.img0;
          img1_synch2_front = Synch2Struct.img1;
          imu_synch_msg_front = Synch2Struct.imu; 

          // Get the next Synch2 message in the deque for use in the while loop (go up to and including the Synch1 msg)
          Synch2Struct_plusOne = Synch2Buffer.at(i+1);
          img0_synch2_msg_plusOne = Synch2Struct_plusOne.img0;

          // Write all Synch2Buffer messages that occur before and including the current Synch1Buffer message to the rosbag
          while(img0_synch2_msg_plusOne.header.stamp != img0_synch2_front.header.stamp) // loop 2
          {
            Synch2Buffer.pop_front();
            // Write any earlier IMU messages that occured before the current Synch2Buffer message to the rosbag
            while(imuBuffer.front().header.stamp != imu_synch_msg_front.header.stamp) // loop 3
            { 
              synched_bag.write(imu_topic, imuBuffer.front().header.stamp, imuBuffer.front());
              imuBuffer.pop_front();
              o += 1;
            }
            
            imuBuffer.pop_front(); // remove the IMU message that is the same as the Synch2Buffer so we don't add it to the rosbag twice

            // Write the Synch2Buffer messages to the rosbag (images and IMU)
            synched_bag.write(imu_topic, imu_synch_msg_front.header.stamp, imu_synch_msg_front);
            synched_bag.write(cam0_topic, img0_synch2_front.header.stamp, img0_synch2_front); 
            synched_bag.write(cam1_topic, img1_synch2_front.header.stamp, img1_synch2_front);
            n += 1;

            // Update the front Synch2Buffer messages
            Synch2Struct = Synch2Buffer.front();
            img0_synch2_front = Synch2Struct.img0;
            img1_synch2_front = Synch2Struct.img1;
            imu_synch_msg_front = Synch2Struct.imu;
          }
          
          // Write the remaining pressure topic of the current Synch1Buffer message to the rosbag
          synched_bag.write(prs_topic, prs_synch1_msg.header.stamp, prs_synch1_msg);
          Synch1Buffer.pop_front();
          m += 1;
          break; 
        }
      }
    }
  }
}



void synchronizeBag(const std::string& filename, ros::NodeHandle& nh)
// Load rosbag, iterate through the messages on each topic, call the synchronizer callback and write to a new bag
{
  // Load unsynched rosbag
  rosbag::Bag unsynched_bag;
  unsynched_bag.open(filename, rosbag::bagmode::Read);
  std::vector<std::string> topics; // create a vector of topics to iterate through
  topics.push_back(cam0_topic);
  topics.push_back(cam1_topic);
  topics.push_back(imu_topic);
  topics.push_back(prs_topic);
  rosbag::View rosbagView(unsynched_bag, rosbag::TopicQuery(topics));
  ROS_INFO("Opening unsynched bag file.");

  // Create empty rosbag to write synched messages into 
  rosbag::Bag synched_bag;
  synched_bag.open(rosbag_folder_path+"/"+"Synched.bag", rosbag::bagmode::Write);  

  // Set up public_simple_filters for message callbacks
  PublicSimpleFilter<sensor_msgs::Image> img0_filter;
  PublicSimpleFilter<sensor_msgs::Image> img1_filter;
  PublicSimpleFilter<sensor_msgs::Imu> imu_filter;
  PublicSimpleFilter<sensor_msgs::FluidPressure> prs_filter;

  // Create Approximate Time Synchronizer 1 (pressure sensor and cameras)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::FluidPressure> approxTimePolicy1; 
  message_filters::Synchronizer<approxTimePolicy1> sync1(approxTimePolicy1(100), img0_filter, img1_filter, prs_filter);
  sync1.registerCallback(boost::bind(&Synch1Callback, _1, _2, _3)); 
 
  // Create Approximate Time Synchronizer 2 (cameras and IMU)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> approxTimePolicy2; 
  message_filters::Synchronizer<approxTimePolicy2> sync2(approxTimePolicy2(100), img0_filter, img1_filter, imu_filter);
  sync2.registerCallback(boost::bind(&Synch2Callback, _1, _2, _3));

  // Register the IMU Buffer Callback
  imu_filter.registerCallback(imuBufferCallback);

  // Iterate through all messages on all topics in the bag and send them to their callbacks
  ROS_INFO("Writing to synched bag file. This may take a few minutes...");
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_filter.publicSignalMessage(img0); // call the Synch1Callback and Synch2Callback
    }
    if (msg.getTopic() == cam1_topic)
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_filter.publicSignalMessage(img1); // call the Synch1Callback and Synch2Callback
    }
    if (msg.getTopic() == imu_topic)
    {
      sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
      if (imu != NULL)
        imu_filter.publicSignalMessage(imu); // call the Synch2Callback and imuBufferCallback
    }
    if (msg.getTopic() == prs_topic)
    {
      sensor_msgs::FluidPressure::ConstPtr prs = msg.instantiate<sensor_msgs::FluidPressure>();
      if (prs != NULL)
        prs_filter.publicSignalMessage(prs); // call the Synch1Callback
    }
    writeToBag(synched_bag); // write to rosbag (disk) and empty the deques as callbacks are made to save RAM space
  }

  unsynched_bag.close();
  synched_bag.close();
  ROS_INFO("Closing both bag files.");
}



//-------------------------MAIN-------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronize_node");
  ros::NodeHandle nh;

  nh.getParam("rosbag_folder_path", rosbag_folder_path);
  nh.getParam("unsynched_bag_name", unsynched_bag_name);
  nh.getParam("cam0_topic", cam0_topic);
  nh.getParam("cam1_topic", cam1_topic);
  nh.getParam("imu_topic", imu_topic);
  nh.getParam("prs_topic", prs_topic);

  synchronizeBag(rosbag_folder_path+"/"+unsynched_bag_name, nh);

  //for loop to print out all the values of the stamp_diffs vectors - are they all zero values? 

  cout << "---" << endl;
  cout << "total messages written to bag:" << endl;
  cout << "imu = " << o << endl;
  cout << "cameras = " << n << endl;
  cout << "pressure = " << m << endl;
  cout << "---" << endl;
  cout << "timestamp differences with respect to cam0:" << endl;
  cout << "max [cam1, imu, prs] = " << *max_element(stamp_diffs_img1.begin(), stamp_diffs_img1.end()) << ", " << *max_element(stamp_diffs_imu.begin(), stamp_diffs_imu.end()) << ", " << *max_element(stamp_diffs_prs.begin(), stamp_diffs_prs.end()) << endl;
  cout << "min [cam1, imu, prs] = " << *min_element(stamp_diffs_img1.begin(), stamp_diffs_img1.end()) << ", " << *min_element(stamp_diffs_imu.begin(), stamp_diffs_imu.end()) << ", " << *min_element(stamp_diffs_prs.begin(), stamp_diffs_prs.end()) << endl;
  cout << "average [cam1, imu, prs] = " << accumulate(stamp_diffs_img1.begin(), stamp_diffs_img1.end(), 0.0)/stamp_diffs_img1.size() << ", " << accumulate(stamp_diffs_img1.begin(), stamp_diffs_img1.end(), 0.0)/stamp_diffs_img1.size() << ", " << accumulate(stamp_diffs_img1.begin(), stamp_diffs_img1.end(), 0.0)/stamp_diffs_img1.size() << endl;
  cout << "---" << endl;
  cout << "Press Ctrl+C to kill the node." << endl;

  ros::spin();
  return 0;
}
