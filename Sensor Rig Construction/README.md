# Sensor Rig Construction
Supplementary material to the sensor rig construction of Adrienne Winter's MSc Mechatronics Engineering dissertation with the African Robotics Unit at the University of Cape Town. Supervised by Robyn Verrinder, Professor Edward Boje and Doctor Paul Amayo.

<img src=https://github.com/adriennewinter/Ship-Hull-Vinyl-Dataset/assets/41785960/88b79a48-5fd5-487f-be79-a25328e091f1 height="300" align=right>

In the 3D printing files directory, all files are created by us except for the ArduCam housing which is provided by ArduCam.

The ROS workspace was developed in ROS1 Melodic and is written in C++. All code is developed by us, except for the following third-party packages:
* xsens_mti_driver - this package is provided by Xsens for use with the IMU.
* ros_deep_learning - this package is provided by Nvidia and is adapted by us for use with the ArduCam CSI cameras on the Nvidia Jetson Nano.

<pre>
 Marine_Vision/
|── 3D Printing Files/ 
│   ├── Box to Strut
│   ├── Case for ArduCam Camera - <i>provided by ArduCam</i>
│   ├── Dive Torch 
│   ├── GoPro Housing to Strut 
│   └── Pole to Strut 
└── ROS_Workspace/ 
    ├── build 
    ├── devel 
    └── src/ 
        ├── ezo_prs - <i>pressure sensor driver</i>
        ├── launch - <i>launch files for managing sensor drivers</i>
        ├── ros_deep_learning - <i>camera driver - provided by Nvidia</i>
        ├── rosbag_manipulations - <i>helper package - splitting and joining rosbags</i>
        ├── synchronize - <i>helper package - synchronize sensor data</i>
        └── xsens_mti_driver - <i>IMU driver - provided by Xsens</i>
</pre>       

