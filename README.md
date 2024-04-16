# Marine_Vision
Supplementary material to Adrienne Winter's MSc Mechatronics Engineering dissertation with the African Robotics Unit at the University of Cape Town.

Supervised by Robyn Verrinder, Professor Edward Boje and Doctor Paul Amayo.

In the 3D printing files directory, all files are created by us except for the ArduCam housing which is provided by ArduCam.

The ROS workspace was developed in ROS1 Melodic and is written in C++. All code is developed by us except for the following:
* xsens_mti_driver - this package is provided by Xsens for use with the IMU.
* ros_deep_learning - this package is provided by Nvidia and is adapted by us for use with the ArduCam CSI cameras on the Nvidia Jetson Nano.

Marine_Vision/
├── 3D Printing Files/
│   ├── Box to Strut
│   ├── Case for ArduCam Camera - _provided by ArduCam_
│   ├── Dive Torch
│   ├── GoPro Housing to Strut
│   └── Pole to Strut
└── ROS_Workspace/
    ├── build
    ├── devel
    └── src/
        ├── ezo_prs - _pressure sensor driver_
        ├── launch - _launch files for managing sensor drivers_
        ├── ros_deep_learning - _camera driver - provided by Nvidia_
        ├── rosbag_manipulations - _helper package - splitting and joining rosbags_
        ├── synchronize - _helper package - synchronize sensor data_
        └── xsens_mti_driver - _IMU driver - provided by Xsens_
