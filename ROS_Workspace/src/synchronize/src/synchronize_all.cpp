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
#include <cmath>

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
#include "synchronize/synchronize_helper.h"
#include "synchronize/synchronize.h"

using namespace std;

//-------------------------MAIN-------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronize_node");

  Synchronize synch_all;

  synch_all.synchronizeBag();

  cout << "---" << endl;
  cout << "total messages written to bag:" << endl;
  cout << "imu = " << synch_all.o << endl;
  cout << "cameras = " << synch_all.n << endl;
  cout << "pressure = " << synch_all.m << endl;
  cout << "---" << endl;
  cout << "timestamp differences with respect to cam0:" << endl;
  cout << "max [cam1, imu, prs] = " << *max_element(synch_all.stamp_diffs_img1.begin(), synch_all.stamp_diffs_img1.end()) << ", " << *max_element(synch_all.stamp_diffs_imu.begin(), synch_all.stamp_diffs_imu.end()) << ", " << *max_element(synch_all.stamp_diffs_prs.begin(), synch_all.stamp_diffs_prs.end()) << " msecs" << endl;
  cout << "min [cam1, imu, prs] = " << *min_element(synch_all.stamp_diffs_img1.begin(), synch_all.stamp_diffs_img1.end()) << ", " << *min_element(synch_all.stamp_diffs_imu.begin(), synch_all.stamp_diffs_imu.end()) << ", " << *min_element(synch_all.stamp_diffs_prs.begin(), synch_all.stamp_diffs_prs.end()) << " msecs" << endl;
  cout << "average [cam1, imu, prs] = " << round(accumulate(synch_all.stamp_diffs_img1.begin(), synch_all.stamp_diffs_img1.end(), 0.0)/synch_all.stamp_diffs_img1.size()) << ", " << round(accumulate(synch_all.stamp_diffs_imu.begin(), synch_all.stamp_diffs_imu.end(), 0.0)/synch_all.stamp_diffs_imu.size()) << ", " << round(accumulate(synch_all.stamp_diffs_prs.begin(), synch_all.stamp_diffs_prs.end(), 0.0)/synch_all.stamp_diffs_prs.size()) << " msecs" << endl;
  cout << "---" << endl;
  cout << "indices of dropped messages:" << endl;
  cout << "cam0 = " << endl;
  printDroppedInds(synch_all.all_stamps_img0, synch_all.written_stamps_img0);
  cout << "cam1 = " << endl;
  printDroppedInds(synch_all.all_stamps_img1, synch_all.written_stamps_img1);
  cout << "imu = " << endl;
  printDroppedInds(synch_all.all_stamps_imu, synch_all.written_stamps_imu); 
  cout << "prs = " << endl;
  printDroppedInds(synch_all.all_stamps_prs, synch_all.written_stamps_prs);
  cout << "---" << endl;
  cout << "Press Ctrl+C to kill the node." << endl;

  return 0;
}