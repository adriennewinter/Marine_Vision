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
        // metrics
        int m, n, o, i, j, k, l;
        std::vector<int> stamp_diffs_imu, stamp_diffs_img1, stamp_diffs_prs;
        std::map<int,ros::Time> all_stamps_imu, all_stamps_img1, all_stamps_img0, all_stamps_prs;
        std::vector<ros::Time> written_stamps_imu, written_stamps_img0, written_stamps_img1, written_stamps_prs;

        Synchronize(const string synchType);
        ~Synchronize();

        // Load rosbag, iterate through the messages on each topic, call the synchronizer callback and write to a new bag
        void synchronizeBag();

    private:
        std::string synch_type_, rosbag_folder_path, unsynched_bag_name, cam0_topic, cam1_topic, imu_topic, prs_topic;

        struct synched_struct {
            sensor_msgs::Image img0;
            sensor_msgs::Image img1;
            sensor_msgs::Imu imu;
            sensor_msgs::FluidPressure prs;
        };

        rosbag::View rosbagView;
        rosbag::Bag unsynched_bag, synched_bag;
        std::deque<sensor_msgs::Imu> imu_buffer;
        std::deque<synched_struct> synch_0_buffer, synch_1_buffer, synch_2_buffer; 

        // Set up public_simple_filters for message callbacks
        PublicSimpleFilter<sensor_msgs::Image> img0_filter, img1_filter;
        PublicSimpleFilter<sensor_msgs::Imu> imu_filter;
        PublicSimpleFilter<sensor_msgs::FluidPressure> prs_filter;

        // Create Approximate Time Synchronizer 0 (stereo cameras)
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> approxTimePolicy0; 
        boost::shared_ptr<message_filters::Synchronizer<approxTimePolicy0>> sync0;

        // Create Approximate Time Synchronizer 1 (pressure sensor and cameras)
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::FluidPressure> approxTimePolicy1; 
        boost::shared_ptr<message_filters::Synchronizer<approxTimePolicy1>> sync1;

        // Create Approximate Time Synchronizer 2 (cameras and IMU)
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> approxTimePolicy2; 
        boost::shared_ptr<message_filters::Synchronizer<approxTimePolicy2>> sync2;

        // Add all IMU messages to a buffer (deque) so that we can preserve higher IMU rate
        void imuBufferCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

        // Callback for synchronizing stereo messages
        void Synch0Callback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg);

        // Callback for synchronizing pressure sensor messages with stereo messages - higher camera rate gets lost
        void Synch1Callback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::FluidPressure::ConstPtr& prs_synch_msg); 

        // Callback for synchronizing stereo messages with IMU messages - higher IMU rate gets lost 
        void Synch2Callback(const sensor_msgs::Image::ConstPtr& img0_synch_msg, const sensor_msgs::Image::ConstPtr& img1_synch_msg, const sensor_msgs::Imu::ConstPtr& imu_synch_msg);       

        // Write synchronized messages to the synched rosbag
        void writeIMU(const sensor_msgs::Imu& imu_synch_msg_front);
        void writeStereoInertial();
        void writeStereo();
        void writeAll();
        void writeToBag();
};

#endif // SYNCHRONIZE_HELPER_H