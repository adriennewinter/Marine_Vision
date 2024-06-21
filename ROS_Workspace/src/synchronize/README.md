# Data Synchronization ROS Package

This ROS package (tested on ROS1 Melodic and Noetic) contains C++ code for synchronizing rosbag data from multiple sensors: stereo cameras, IMU and a pressure sensor.

The '[Approximate Time Synchronizer](http://wiki.ros.org/message_filters/ApproximateTime)' filter is used from the ROS '[message_filters](http://wiki.ros.org/message_filters)' package to synchronize messages. This filter outputs synchronized messages at the lowest sensor frequency. To preserve the higher sensor frequencies, a series of buffers are used.

## Sensor Data Types
The camera data is expected to be of the type "[sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)".

The IMU data is expected to be of the type "[sensor_msgs/IMU](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)".

The pressure sensor data is expected to be of the type "[sensor_msgs/FluidPressure](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/FluidPressure.html)".

## Usage
You will need to edit the topic names and bag file name in the params.yaml file found in the config folder before calling the launch file for the node. 

The launch file creates a synchronizer node that opens a pre-recorded rosbag and extracts all messages and their data to perform synchronization using ROS timestamps. The synchronizer node expects that all data is recorded into one rosbag under different topics.

```
$ roslaunch synchronize synchronize_node.launch
```

It is normal that some messages can be dropped, based on the best-match algorithm in the Approximate Time Synchronizer filter. This ROS package also drops any messages that occur after the last low-frequency sensor message. This means that an IMU with 100Hz will drop the last 100 or less messages after the last message from a 1Hz sensor.

## Installation
Download the 'synchronize' package into the src folder of your ROS workspace. 
```
$ cd root/folder/ROS/Workspace
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="synchronize"
```
Using the flag `-DCATKIN_WHITELIST_PACKAGES="synchronize"` will build just this package and not your whole workspace. If you use this flag you will need to set it back to `-DCATKIN_WHITELIST_PACKAGES=" "` next time you build if you want to re-build your whole workspace.
