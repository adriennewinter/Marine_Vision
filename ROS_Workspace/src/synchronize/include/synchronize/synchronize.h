#ifndef SYNCHRONIZE_H
#define SYNCHRONIZE_H

#include <cstdio>
#include <cmath>
#include <sensor_msgs/Image.h> // camera image messages

using namespace std;

template <typename M>
  
int findStampDiffMsec(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const M synch_msg){
  // finds the difference between the cam0 timestamp and another topic message and converts to milliseconds
  long img0_stamp_sec = img0_synch_msg->header.stamp.sec;
  long img0_stamp_nsec = img0_synch_msg->header.stamp.nsec;
  //cout << "img0_stamp_sec = " << img0_stamp_sec << endl;

  long stamp_sec = synch_msg->header.stamp.sec;
  long stamp_nsec = synch_msg->header.stamp.nsec;
  //cout << "stamp_sec = " << stamp_sec << endl;

  long diff_sec = abs(img0_stamp_sec - stamp_sec);
  long diff_nsec = abs(img0_stamp_nsec - stamp_nsec);
  //cout << "diff_nsec = " << diff_nsec << endl;

  int diff_msec = abs(diff_sec*1000 + diff_nsec/1000000);
  //cout << "diff_msec = " << diff_msec << endl;
  return diff_msec;
}


#endif // SYNCHRONIZE_H