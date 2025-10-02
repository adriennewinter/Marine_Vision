# Data Synchronisation ROS Package

## Synchronisation
Good synchronisation is imperative for visual odometry and SLAM pipelines. If the time difference between two camera streams on a moving body is too large, the disparity value between matched features will be incorrect, leading to larger errors in scene depth and odometry estimations. 

If there is no possibility of hardware-level synchronisation, we can synchronise data in software. One way of doing this is by using timestamps that have been allocated to sensor data messages by a central processor that has a single clock signal. If timestamps are allocated to data streams from decentralised processors or chips, their clock signals could be out of synchronism because of clock drift and skew, adding a layer of complexity to the synchronisation.

<br>
<p align="center" width="100%">
  <img width="70%" alt="clocks_out_of_synch" src="https://github.com/user-attachments/assets/ce8cd2d8-708a-434f-89d4-0ee385f48309" />
  <br>
  <i>Drift (top) and skew (bottom) of separate clock signals.</i>
</p>

When the clock signals between two stereo cameras have drift or skew and there is movement of the robot body during the time difference when the second camera takes its image, the position of a feature in the second image will be shifted slightly, affecting the disparity of the feature between left and right images.

<br>
<p align="center" width="100%">
  <img width="70%" alt="Synch_Depth_Problem" src="https://github.com/user-attachments/assets/a99286d2-7247-4fdc-af24-c064b59f38e6" />
  <br>
  <i>(right) The grey star in the right image is where the feature should be positioned if the clocks were synchronised. The orange star in the right image is the feature position when the clocks have drift or skew and the platform body has movement during this time.</i>
</p>

## Package Implementation
This ROS package (tested on ROS1 Melodic and Noetic) contains C++ code for synchronising rosbag data from multiple sensors: stereo cameras, IMU and a pressure sensor (but can be adapted to other sensor types).

The '[Approximate Time synchroniser](http://wiki.ros.org/message_filters/ApproximateTime)' filter is used from the ROS '[message_filters](http://wiki.ros.org/message_filters)' package to synchronise messages. Because it is approximate time, it does not expect the timestamps to be exact, instead, it does a best-match search over the message header timestamps to find sets of synchronised messages across the input topics. Some messages can be dropped during this process when optimizing for the best fit over multiple messages. 

When synchronising sensors with different sampling rates, the Approximate Time synchronizer filter outputs synchronised messages at the lowest sensor frequency. To preserve the higher frequencies, we implement a series of buffers. The buffers save all messages that occur between the synchronised messages, preserving the original frequency of higher frequency topics. 

<br>
<p align="center" width="100%">
  <img width="50%" alt="Synch_Code_Buffers" src="https://github.com/user-attachments/assets/b9d17649-c872-4f22-87ee-d03a89fc8bfa" />
  <br>
  <i>For every message in the lower frequency sensor topic (pressure), the ROS Approximate Time synchronizer filter will find a matching message in the higher frequency topics (shown in blue). We implement buffers to save higher-frequency messages on other topics (shown in white on the camera and IMU).</i>
</p>

<p align="center" width="100%">
  <img width="40%" alt="synchronization_code_implementation" src="https://github.com/user-attachments/assets/a1c4c714-4af6-4d3e-93d3-c1c5be00c886" />
  <br>
  <i>Diagram showing the logic of the synchronisation package, where the output is a synchronised rosbag with original sensor frequencies preserved.</i>
</p>

## Sensor Data Types
The camera data is expected to be of the type "[sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)".

The IMU data is expected to be of the type "[sensor_msgs/IMU](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)".

The pressure sensor data is expected to be of the type "[sensor_msgs/FluidPressure](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/FluidPressure.html)".

## Usage
You will need to edit the topic names and bag file name in the params.yaml file found in the config folder before calling the launch file for the node. 

The launch file creates a synchroniser node that opens a pre-recorded rosbag and extracts all messages and their data to perform synchronisation using ROS timestamps. The synchroniser node expects that all data is recorded into one rosbag under different topics.

```
$ roslaunch synchronise synchronise_node.launch
```

It is normal that some messages can be dropped, based on the best-match algorithm in the Approximate Time synchroniser filter. This ROS package also drops any messages that occur after the last low-frequency sensor message. This means that an IMU with 100Hz will drop the last 100 or less messages after the last message from a 1Hz sensor.

## Installation
Download the 'synchronise' package into the src folder of your ROS workspace. 
```
$ cd root/folder/ROS/Workspace
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="synchronise"
```
Using the flag `-DCATKIN_WHITELIST_PACKAGES="synchronise"` will build just this package and not your whole workspace. If you use this flag you will need to set it back to `-DCATKIN_WHITELIST_PACKAGES=" "` next time you build if you want to re-build your whole workspace.
