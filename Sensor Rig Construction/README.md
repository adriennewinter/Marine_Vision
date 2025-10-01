# Sensor Rig Construction
Supplementary material to the sensor rig construction used to create the Shiphull Vinyl underwater SLAM dataset.  

<pre>
Marine_Vision/Sensor Rig Construction
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

In the 3D printing files directory, all files are created by us except for the ArduCam housing which is provided by ArduCam.  

<p align="center" width="100%">
 <img width="10%" alt="goPro-to-strut" src="https://github.com/user-attachments/assets/7478ebf5-0220-4198-b137-d3c0060c9056" />
 <img width="10%" alt="pole-to-strut" src="https://github.com/user-attachments/assets/53ab0e90-5454-4c01-b76f-f616a157aa2c" />
 <img width="7%" alt="dive_torch" src="https://github.com/user-attachments/assets/c0b21d17-aede-48c0-9504-da6bcca11c91" />
 <br>
 3D printed parts designed for the data collection rig.
</p>

<p align="center" width="100%"> 
 <img width="30%" alt="arducam-casing" src="https://github.com/user-attachments/assets/1355826c-f38b-427d-83a0-ed2508d60591" />  
 <br />
 The two-part camera casing 3D printing model provided by ArduCam. 
</p>

The ROS workspace was developed in ROS1 Melodic and is written in C++. All code is developed by us, except for the following third-party packages:
* xsens_mti_driver - this package is provided by Xsens for use with the IMU.
* ros_deep_learning - this package is provided by Nvidia and is adapted by us for use with the ArduCam CSI cameras on the Nvidia Jetson Nano.

<p align="center">
 <img width="60%" alt="final_rig" src="https://github.com/user-attachments/assets/8583c64e-fdea-493d-bdfc-fcfca55992ff" />   
 <br>
 Image showing the final prototype sensor rig. Where: (a) ArduCam CSI camera
 inside a GoPro waterproof housing, (b) central rig strut, (c) 3D printed camera-housing-to-strut
 connector, (d) 3D printed strut-to-sensor-box connector, (e) 3D printed torch-to-strut connector, (f)
 dive torch, (g) epoxy-resin filled sensor housing containing (h) pressure sensor, (not visible) IMU,
 Jetson Nano and buck converter.
</p>

<br>

<p align="center">
 <img width="60%" alt="rig_electrical_diagram" src="https://github.com/user-attachments/assets/ea3806b2-2c6c-4d69-9654-9fa6b12294c0" />  
 <br>
 Diagram showing the electrical setup. 
</p>
  
<br>

<p align="center">
 <img width="60%" alt="ROS_development" src="https://github.com/user-attachments/assets/3a795680-e548-4dcf-89fa-fb44ff74cee8" />
</p>
