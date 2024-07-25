#include <cstdio>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Image.h> // camera image messages

#include "synchronize/synchronize_helper.h"

using namespace std;

void printDroppedInds(const std::map<int,ros::Time>& all_stamps, const std::vector<ros::Time>& written_stamps){
  std::vector<int> dropped_inds;
  
  // Iterate through the all_stamps map
  for(const auto& map_point : all_stamps){
      // Check if the timestamp (map_point.second) is not in written_stamps - this is a dropped message
      if(std::find(written_stamps.begin(), written_stamps.end(), map_point.second) == written_stamps.end()){
          dropped_inds.push_back(map_point.first); 
      }
  }

  // Print dropped inds
  sort(dropped_inds.begin(), dropped_inds.end());
  for(int ind : dropped_inds){ cout << ind << "  "; }
  cout << endl;
}