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
#include "synchronize/synchronize_helper.h"
#include "synchronize/synchronize.h"

using namespace std;

int main(int argc, char** argv)
{
  // Create synchronize ROS node
  ros::init(argc, argv, "stereo_synch");

  Synchronize stereo_synch;

  stereo_synch.synchronizeBag();

  cout << "---" << endl;
  cout << "Total synched camera messages written to bag = " << stereo_synch.n << endl;
  cout << "---" << endl;
  cout << "cam1 timestamp differences with respect to cam0:" << endl;
  cout << "max = " << *max_element(stereo_synch.stamp_diffs_img1.begin(), stereo_synch.stamp_diffs_img1.end()) << " msecs" << endl;
  cout << "min = " << *min_element(stereo_synch.stamp_diffs_img1.begin(), stereo_synch.stamp_diffs_img1.end()) << " msecs" << endl;
  cout << "average = " << round(accumulate(stereo_synch.stamp_diffs_img1.begin(), stereo_synch.stamp_diffs_img1.end(), 0.0)/stereo_synch.stamp_diffs_img1.size()) << " msecs" << endl;
  cout << "---" << endl;
  cout << "indices of dropped messages:" << endl;
  cout << "cam0 = " << endl;
  printDroppedInds(stereo_synch.all_stamps_img0, stereo_synch.written_stamps_img0);
  cout << "cam1 = " << endl;
  printDroppedInds(stereo_synch.all_stamps_img1, stereo_synch.written_stamps_img1);
  cout << "---" << endl;
  cout << "Press Ctrl+C to kill the node." << endl;
 
  return 0;
}