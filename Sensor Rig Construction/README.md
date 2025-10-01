# Sensor Rig Construction
## Repo Structure
Supplementary material to the sensor rig construction used to create the [Shiphull Vinyl underwater SLAM dataset](https://github.com/African-Robotics-Unit/Ship-Hull-Vinyl-Dataset).  

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
 <img width="7.5%" alt="L-bracket" src="https://github.com/user-attachments/assets/16da769c-b641-4a1e-8764-5287c46f71f9" />
 <br>
 3D printed parts designed in SolidWorks for the data collection rig.
</p>

<p align="center" width="100%"> 
 <img width="30%" alt="arducam-casing" src="https://github.com/user-attachments/assets/1355826c-f38b-427d-83a0-ed2508d60591" />  
 <br />
 The two-part camera casing 3D printing model provided by ArduCam. 
</p>

The ROS workspace was developed in ROS1 Melodic and is written in C++. All code is developed by us, except for the following third-party packages:
* xsens_mti_driver - this package is provided by Xsens for use with the IMU.
* ros_deep_learning - this package is provided by Nvidia and is adapted by us for use with the ArduCam CSI cameras on the Nvidia Jetson Nano.

<br>
<p align="center">
 <img width="60%" alt="ROS_development" src="https://github.com/user-attachments/assets/e5e8ab37-084c-4c2b-9c5d-1963f4df7593" />
 <br>   
 Image showing development of the ROS workspace and nodes on the sensor rig.   
</p>

# Prototype Development Details
The rig was designed with practical constraints in mind, focusing on affordability, portability, and compatibility with key sensors such as stereo cameras, an IMU, and a pressure sensor. These components were selected for their complementary roles in underwater SLAM and their alignment with the research objectives of enabling accurate trajectory estimation and visual reconstructions.

Because the planned dataset collection and evaluation of the calibration method are planned to be taken in a tank, pool and in shallow sea water off the side of a jetty, the depth rating of the prototype sensor rig is 2 m. These use cases also guide our decision to make the rig handheld and operated from the side of the pool, tank or jetty.

GoPro Hero 10 waterproof housings are used for the camera housings, as they were low-cost and sized efficiently for the ArduCam CSI M12 lens cameras.

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
Data rates were initially lower than expected for the cameras. The CPU, GPU and RAM usage was monitored during operation, showing the RAM reaching capacity. A reduction in image size and fps from 1280x720 to 600x600 pixels and 15 to 10 fps resulted in expected data rates and no RAM overflow.

<p align="center">
 <img width="60%" alt="rig_electrical_diagram" src="https://github.com/user-attachments/assets/ea3806b2-2c6c-4d69-9654-9fa6b12294c0" />  
 <br>
 Diagram showing the electrical setup. 
</p>

A buck converter from 12 V power bank to 5 V on the Jetson Nano is used to ensure a stable 5 V supply and no current throttling from under-voltage protection on the Jetson.

The data collection process is controlled and monitored over an SSH connection via an Ethernet cable between a host laptop and the Jetson Nano. We use SSH X11 forwarding over Putty between a Windows host computer and the Jetson to view the camera streams with ROS rqt_gui.

ROS (Robot Operating System) is used on the Jetson Nano for data collection between the peripherals - an IMU, pressure sensor and two CSI cameras. ROS Melodic was chosen due to the Jetson Nano Nvidia Jetpack OS being based on Ubuntu 18.04. The Xsens IMU is supported with maintained ROS drivers. The CSI cameras use an edited version of the camera driver from the Nvidia ros_deep_learning GitHub repository.
