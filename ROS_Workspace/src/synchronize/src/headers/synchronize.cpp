#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h> // camera image messages
#include <sensor_msgs/Imu.h> // IMU messages
#include <sensor_msgs/FluidPressure.h> // pressure sensor messages
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/simple_filter.h>

#include "synchronize/synchronize.h"
#include "synchronize/synchronize_helper.h"
#include "synchronize/public_simple_filter.h"

using namespace std;

Synchronize::Synchronize() {
    ros::NodeHandle nh;
    nh.getParam("rosbag_folder_path", rosbag_folder_path);
    nh.getParam("unsynched_bag_name", unsynched_bag_name);
    nh.getParam("cam0_topic", cam0_topic);
    nh.getParam("cam1_topic", cam1_topic);
    nh.getParam("imu_topic", imu_topic);
    nh.getParam("prs_topic", prs_topic);

    m  = 0; n = 0; o = 0;
    i = 1; j = 1; k = 1; l = 1;

    // Load unsynched rosbag
    unsynched_bag.open(rosbag_folder_path+"/"+unsynched_bag_name, rosbag::bagmode::Read);
    vector<string> topics; // create a vector of topics to iterate through
    topics.push_back(cam0_topic);
    topics.push_back(cam1_topic);
    topics.push_back(imu_topic);
    topics.push_back(prs_topic);
    rosbagView.addQuery(unsynched_bag, rosbag::TopicQuery(topics));
    cout << "Opening unsynched bag file." << endl;

    // Create empty rosbag to write synched messages into 
    synched_bag.open(rosbag_folder_path+"/"+"Synched.bag", rosbag::bagmode::Write);  

    // Register Approximate Time Synchronizer filter callbacks
    sync1 = boost::make_shared<message_filters::Synchronizer<approxTimePolicy1>> (approxTimePolicy1(100), img0_filter, img1_filter, prs_filter);
    sync1->registerCallback(boost::bind(&Synchronize::Synch1Callback, this, _1, _2, _3)); 
    sync2 = boost::make_shared<message_filters::Synchronizer<approxTimePolicy2>> (approxTimePolicy2(100), img0_filter, img1_filter, imu_filter);
    sync2->registerCallback(boost::bind(&Synchronize::Synch2Callback, this, _1, _2, _3));

    // Register the IMU Buffer Callback
    imu_filter.registerCallback(boost::bind(&Synchronize::imuBufferCallback, this, _1));
}
// -----------------------------------------------------------------------------------------
void Synchronize::imuBufferCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    imu_buffer.push_back(*imu_msg);
}
// -----------------------------------------------------------------------------------------
void Synchronize::Synch1Callback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::FluidPressure::ConstPtr& prs_synch_msg)
{ 
    struct synched_struct SynchedMsgsStruct;

    // Insert synched messages into the struct 
    SynchedMsgsStruct.img0 = *img0_synch_msg;
    SynchedMsgsStruct.img1 = *img1_synch_msg;
    SynchedMsgsStruct.prs = *prs_synch_msg;

    // Insert the struct into the deque
    synch_1_buffer.push_back(SynchedMsgsStruct);

    // Find timestamp differences with respect to img0 and add to respective vectors
    int prs_diff = findStampDiffMsec(img0_synch_msg, prs_synch_msg);
    stamp_diffs_prs.push_back(prs_diff);
}
// -----------------------------------------------------------------------------------------
void Synchronize::Synch2Callback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::Imu::ConstPtr& imu_synch_msg)
{ 
    struct synched_struct SynchedMsgsStruct;

    // Insert synched messages into the struct
    SynchedMsgsStruct.img0 = *img0_synch_msg;
    SynchedMsgsStruct.img1 = *img1_synch_msg;
    SynchedMsgsStruct.imu = *imu_synch_msg;

    // Insert the struct into the deque
    synch_2_buffer.push_back(SynchedMsgsStruct);

    // Find timestamp differences with respect to img0 and add to respective vectors
    int img1_diff = findStampDiffMsec(img0_synch_msg, img1_synch_msg);
    int imu_diff = findStampDiffMsec(img0_synch_msg, imu_synch_msg);
    stamp_diffs_img1.push_back(img1_diff);
    stamp_diffs_imu.push_back(imu_diff);
}
// -----------------------------------------------------------------------------------------
void Synchronize::writeToBag()
{
  sensor_msgs::Image img0_synch1_msg, img1_synch1_msg, img0_synch2_msg, img0_synch2_front, img1_synch2_front, img0_synch2_msg_plusOne;
  sensor_msgs::Imu imu_synch_msg_front, imu_synch2_msg;
  sensor_msgs::FluidPressure prs_synch1_msg;
  struct synched_struct Synch1Struct, Synch2Struct, Synch2Struct_plusOne;

  if(!synch_1_buffer.empty())
  {
    // Get latest synched message struct from the synch_1_buffer
    Synch1Struct = synch_1_buffer.front();
    img0_synch1_msg = Synch1Struct.img0;
    img1_synch1_msg = Synch1Struct.img1;
    prs_synch1_msg = Synch1Struct.prs;
    
    // Find the message in synch_2_buffer that equals the current synch_1_buffer message
    for(int i=0; i<synch_2_buffer.size(); i++) // loop 1
    {
      // Get the current img0 message from the synch_2_buffer for comparison with synch_1_buffer
      Synch2Struct = synch_2_buffer.at(i);
      img0_synch2_msg = Synch2Struct.img0;

      // If the current Synch2 message is equal to the current Synch1 message 
      if(img0_synch1_msg.header.stamp == img0_synch2_msg.header.stamp)
      {
        if(synch_2_buffer.size() > (i+2)) 
        {
          // Get the front message from synch_2_buffer 
          Synch2Struct = synch_2_buffer.front();
          img0_synch2_front = Synch2Struct.img0;
          img1_synch2_front = Synch2Struct.img1;
          imu_synch_msg_front = Synch2Struct.imu; 

          // Get the next Synch2 message in the deque for use in the while loop (go up to and including the Synch1 msg)
          Synch2Struct_plusOne = synch_2_buffer.at(i+1);
          img0_synch2_msg_plusOne = Synch2Struct_plusOne.img0;

          // Write all synch_2_buffer messages that occur before and including the current synch_1_buffer message to the rosbag
          while(img0_synch2_msg_plusOne.header.stamp != img0_synch2_front.header.stamp) // loop 2
          {
            synch_2_buffer.pop_front();
            // Write any earlier IMU messages that occured before the current synch_2_buffer message to the rosbag
            while(imu_buffer.front().header.stamp != imu_synch_msg_front.header.stamp) // loop 3
            { 
              synched_bag.write(imu_topic, imu_buffer.front().header.stamp, imu_buffer.front());
              written_stamps_imu.push_back(imu_buffer.front().header.stamp); // Store to find indices of dropped messages
              imu_buffer.pop_front();
              o++;
            }
            imu_buffer.pop_front(); // remove the IMU message that is the same as the synch_2_buffer so we don't add it to the rosbag twice

            // Write the synch_2_buffer messages to the rosbag (images and IMU)
            synched_bag.write(imu_topic, img0_synch2_front.header.stamp, imu_synch_msg_front);
            synched_bag.write(cam0_topic, img0_synch2_front.header.stamp, img0_synch2_front); 
            synched_bag.write(cam1_topic, img0_synch2_front.header.stamp, img1_synch2_front);
            n++; o++;

            // Store to find indices of dropped messages
            written_stamps_imu.push_back(imu_synch_msg_front.header.stamp);
            written_stamps_img0.push_back(img0_synch2_front.header.stamp);
            written_stamps_img1.push_back(img1_synch2_front.header.stamp);

            // Update the front synch_2_buffer messages
            Synch2Struct = synch_2_buffer.front();
            img0_synch2_front = Synch2Struct.img0;
            img1_synch2_front = Synch2Struct.img1;
            imu_synch_msg_front = Synch2Struct.imu;
          }
          
          // Write the remaining pressure topic of the current synch_1_buffer message to the rosbag
          synched_bag.write(prs_topic, img0_synch2_front.header.stamp, prs_synch1_msg);
          written_stamps_prs.push_back(prs_synch1_msg.header.stamp); // Store to find indices of dropped messages
          synch_1_buffer.pop_front();
          m++;

          break; 
        }
      }
    }
  }
}
// -----------------------------------------------------------------------------------------
void Synchronize::synchronizeBag()
{
  // Iterate through all messages on all topics in the bag and send them to their callbacks
  cout << "Writing to synched bag file. This may take a few minutes..." << endl;
  BOOST_FOREACH(rosbag::MessageInstance const msg, rosbagView)
  {
    if (msg.getTopic() == cam0_topic)
    {
      sensor_msgs::Image::ConstPtr img0 = msg.instantiate<sensor_msgs::Image>();
      if (img0 != NULL)
        img0_filter.publicSignalMessage(img0); // call the Synch1Callback and Synch2Callback
        all_stamps_img0[i] = img0->header.stamp; // record message index and timestamp 
        i++;
    }
    if (msg.getTopic() == cam1_topic)
    {
      sensor_msgs::Image::ConstPtr img1 = msg.instantiate<sensor_msgs::Image>();
      if (img1 != NULL)
        img1_filter.publicSignalMessage(img1); // call the Synch1Callback and Synch2Callback
        all_stamps_img1[j] = img1->header.stamp; // record message index and timestamp 
        j++;
    }
    if (msg.getTopic() == imu_topic)
    {
      sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
      if (imu != NULL)
        imu_filter.publicSignalMessage(imu); // call the Synch2Callback and imuBufferCallback
        all_stamps_imu[k] = imu->header.stamp; // record message index and timestamp  
        k++;
    }
    if (msg.getTopic() == prs_topic)
    {
      sensor_msgs::FluidPressure::ConstPtr prs = msg.instantiate<sensor_msgs::FluidPressure>();
      if (prs != NULL)
        prs_filter.publicSignalMessage(prs); // call the Synch1Callback
        all_stamps_prs[l] = prs->header.stamp; // record message index and timestamp 
        l++;
    }
    writeToBag(); // write to rosbag (disk) and empty the deques as callbacks are made to save RAM space
  }
}
// -----------------------------------------------------------------------------------------
Synchronize::~Synchronize(){
    unsynched_bag.close();
    synched_bag.close();
}