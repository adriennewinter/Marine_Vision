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
#include "synchronize/synchronize_helper.h"
#include "synchronize/synchronize.h"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_inertial_synch");

  Synchronize ster_inert_synch;

  ster_inert_synch.synchronizeBag();

  cout << "---" << endl;
  cout << "total messages written to bag:" << endl;
  cout << "imu = " << ster_inert_synch.o << endl;
  cout << "cameras = " << ster_inert_synch.n << endl;
  cout << "---" << endl;
  cout << "timestamp differences with respect to cam0:" << endl;
  cout << "max [cam1, imu] = " << *max_element(ster_inert_synch.stamp_diffs_img1.begin(), ster_inert_synch.stamp_diffs_img1.end()) << ", " << *max_element(ster_inert_synch.stamp_diffs_imu.begin(), ster_inert_synch.stamp_diffs_imu.end()) << ", " << "msecs" << endl;
  cout << "min [cam1, imu] = " << *min_element(ster_inert_synch.stamp_diffs_img1.begin(), ster_inert_synch.stamp_diffs_img1.end()) << ", " << *min_element(ster_inert_synch.stamp_diffs_imu.begin(), ster_inert_synch.stamp_diffs_imu.end()) << ", " << "msecs" << endl;
  cout << "average [cam1, imu] = " << round(accumulate(ster_inert_synch.stamp_diffs_img1.begin(), ster_inert_synch.stamp_diffs_img1.end(), 0.0)/ster_inert_synch.stamp_diffs_img1.size()) << ", " << round(accumulate(ster_inert_synch.stamp_diffs_imu.begin(), ster_inert_synch.stamp_diffs_imu.end(), 0.0)/ster_inert_synch.stamp_diffs_imu.size()) << ", " << "msecs" << endl;
  cout << "---" << endl;
  cout << "indices of dropped messages:" << endl;
  cout << "cam0 = " << endl;
  printDroppedInds(ster_inert_synch.all_stamps_img0, ster_inert_synch.written_stamps_img0);
  cout << "cam1 = " << endl;
  printDroppedInds(ster_inert_synch.all_stamps_img1, ster_inert_synch.written_stamps_img1);
  cout << "imu = " << endl;
  printDroppedInds(ster_inert_synch.all_stamps_imu, ster_inert_synch.written_stamps_imu); 
  cout << "---" << endl;
  cout << "Press Ctrl+C to kill the node." << endl;

  return 0;
}