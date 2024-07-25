#ifndef SYNCHRONIZE_H
#define SYNCHRONIZE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h> // camera image messages
#include <sensor_msgs/Imu.h> // IMU messages
#include <sensor_msgs/FluidPressure.h> // pressure sensor messages

#include "synchronize/public_simple_filter.h"

using namespace std;

class Synchronize { 
    public:        
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

        rosbag::View rosbagView;
        rosbag::Bag unsynched_bag, synched_bag;
        std::deque<sensor_msgs::Imu> imu_buffer;
        std::deque<synched_struct> synch_1_buffer, synch_2_buffer; 

        // metrics
        int m, n, o; 
        int i, j, k, l;
        std::vector<int> stamp_diffs_imu, stamp_diffs_img1, stamp_diffs_prs;
        std::map<int,ros::Time> all_stamps_imu, all_stamps_img1, all_stamps_img0, all_stamps_prs;
        std::vector<ros::Time> written_stamps_imu, written_stamps_img0, written_stamps_img1, written_stamps_prs;

        // Set up public_simple_filters for message callbacks
        PublicSimpleFilter<sensor_msgs::Image> img0_filter;
        PublicSimpleFilter<sensor_msgs::Image> img1_filter;
        PublicSimpleFilter<sensor_msgs::Imu> imu_filter;
        PublicSimpleFilter<sensor_msgs::FluidPressure> prs_filter;

        Synchronize();
        ~Synchronize();

        // Load rosbag, iterate through the messages on each topic, call the synchronizer callback and write to a new bag
        void synchronizeBag();

    private:
        // Add all IMU messages to a buffer (deque) so that we can preserve higher IMU rate
        void imuBufferCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

        // Callback for synchronizing pressure sensor messages with stereo messages - higher camera rate gets lost
        void Synch1Callback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::FluidPressure::ConstPtr& prs_synch_msg); 

        // Callback for synchronizing stereo messages with IMU messages - higher IMU rate gets lost 
        void Synch2Callback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::Imu::ConstPtr& imu_synch_msg);       

        // Write synchronized messages to the synched rosbag
        void writeToBag();
};

#endif // SYNCHRONIZE_HELPER_H