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
// Installation and Usage:
// Save this 'synchronize' package to your ROS workspace src folder and build it with catkin.
//
// You may have to remove the "protected:" above the signalMessage function in simple_filter.h
// $ sudo gedit /opt/ros/melodic/include/message_filters/simple_filter.h
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

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/simple_filter.h>
#include <sensor_msgs/CompressedImage.h> // compressed camera image messages
#include <sensor_msgs/Image.h> // camera image messages
#include <sensor_msgs/Imu.h> // IMU messages
#include <sensor_msgs/FluidPressure.h> // pressure sensor messages
 
using namespace std;

//-------------------------GLOBAL VARIABLES-----------------------------------------------------
std::string rosbagFolderPath;
std::string unsynchedBagName;
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

int synch1_cnt, synch2_cnt, imuBuff_cnt, m, n, o = 0; 
std::deque<synched_struct> Synch1Buffer, Synch2Buffer; // a deque of structs
std::deque<sensor_msgs::Imu> imuBuffer;
std::deque<sensor_msgs::FluidPressure> prsBuffer;

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

  synch1_cnt += 1;
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

  synch2_cnt += 1;
}


void imuBufferCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
// Add all IMU messages to a buffer (deque) so that we can preserve higher IMU rate
{
  imuBuffer.push_back(*imu_msg);

  imuBuff_cnt += 1;
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
              o += 1;
              imuBuffer.pop_front();
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
  synched_bag.open(rosbagFolderPath+"/"+"Synched.bag", rosbag::bagmode::Write); 

  // Set up message_filters subscribers to capture messages from the bag
  message_filters::Subscriber<sensor_msgs::Image> img0_sub(nh, cam0_topic, 10); 
  message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, cam1_topic, 10);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, imu_topic, 20); 
  message_filters::Subscriber<sensor_msgs::FluidPressure> prs_sub(nh, prs_topic, 10); 

  // Create Approximate Time Synchronizer 1 (pressure sensor and cameras)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::FluidPressure> approxTimePolicy1; 
  message_filters::Synchronizer<approxTimePolicy1> sync1(approxTimePolicy1(100), img0_sub, img1_sub, prs_sub);
  sync1.registerCallback(boost::bind(&Synch1Callback, _1, _2, _3)); 
 
  // Create Approximate Time Synchronizer 2 (cameras and IMU)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> approxTimePolicy2; 
  message_filters::Synchronizer<approxTimePolicy2> sync2(approxTimePolicy2(100), img0_sub, img1_sub, imu_sub);
  sync2.registerCallback(boost::bind(&Synch2Callback, _1, _2, _3));

  // Register the IMU Buffer Callback
  imu_sub.registerCallback(imuBufferCallback);

  // Iterate through all messages on all topics in the bag and send them to their callbacks
  ROS_INFO("Writing to synched bag file. This will take a few minutes...");
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_sub.signalMessage(img0); // call the Synch1Callback and Synch2Callback
    }
    if (msg.getTopic() == cam1_topic)
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_sub.signalMessage(img1); // call the Synch1Callback and Synch2Callback
    }
    if (msg.getTopic() == imu_topic)
    {
      sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
      if (imu != NULL)
        imu_sub.signalMessage(imu); // call the Synch2Callback and imuBufferCallback
    }
    if (msg.getTopic() == prs_topic)
    {
      sensor_msgs::FluidPressure::ConstPtr prs = msg.instantiate<sensor_msgs::FluidPressure>();
      if (prs != NULL)
        prs_sub.signalMessage(prs); // call the Synch1Callback
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

  nh.getParam("rosbagFolderPath", rosbagFolderPath);
  nh.getParam("unsynchedBagName", unsynchedBagName);
  nh.getParam("cam0_topic", cam0_topic);
  nh.getParam("cam1_topic", cam1_topic);
  nh.getParam("imu_topic", imu_topic);
  nh.getParam("prs_topic", prs_topic);

  synchronizeBag(rosbagFolderPath+"/"+unsynchedBagName, nh);

  ROS_INFO("Pressure sensor and camera synched messages (Synch1Buffer) = %d", synch1_cnt);
  ROS_INFO("Camera and IMU synched messages (Synch2Buffer) = %d", synch2_cnt);
  ROS_INFO("IMU messages added to imuBuffer = %d", imuBuff_cnt);
  ROS_INFO("total imuBuffer messages written to bag = %d", o);
  ROS_INFO("total Synch2Buffer messages written to bag = %d", n);
  ROS_INFO("total Synch1Buffer messages written to bag = %d", m);
  ROS_INFO("Synch2Buffer.size() = %lu",Synch2Buffer.size());
  ROS_INFO("imuBuffer.size() = %lu",imuBuffer.size());
  ROS_INFO("Press Ctrl+C to kill the node.");

  ros::spin();
  return 0;
}
