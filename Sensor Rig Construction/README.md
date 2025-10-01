# Sensor Rig Construction
## Directory Structure
Supplementary material to the sensor rig construction used to create the Shiphull Vinyl underwater [SLAM dataset](https://github.com/African-Robotics-Unit/Ship-Hull-Vinyl-Dataset).  

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
 <i>3D printed connector parts designed in SolidWorks for the data collection rig.</i>
</p>

<p align="center" width="100%"> 
 <img width="30%" alt="arducam-casing" src="https://github.com/user-attachments/assets/1355826c-f38b-427d-83a0-ed2508d60591" />  
 <br />
 <i>The two-part camera casing 3D printing model provided by ArduCam.</i>
</p>

The ROS workspace was developed in ROS1 Melodic and is written in C++. All code is developed by us, except for the following third-party packages:
* xsens_mti_driver - this package is provided by Xsens for use with the IMU.
* ros_deep_learning - this package is provided by Nvidia and is adapted by us for use with the ArduCam CSI cameras on the Nvidia Jetson Nano.

The custom ROS data synchronisation package is also provided in the ROS workspace [here](https://github.com/adriennewinter/Marine_Vision/tree/main/Sensor%20Rig%20Construction/ROS_Workspace/src/synchronize).

# Prototype Development Details
The rig was designed with practical constraints in mind, focusing on affordability, portability, and compatibility with key sensors such as stereo cameras, an IMU, and a pressure sensor. These components were selected for their complementary roles in underwater SLAM and their alignment with the research objectives of enabling accurate trajectory estimation and visual reconstructions.

Data collection was planned to be operated from the side of a pool, tank or jetty in shallow and calm water. These use cases guide the decision to design the rig to be handheld with a depth rating of 2 m.

<br>
<p align="center">
 <img width="60%" alt="final_rig" src="https://github.com/user-attachments/assets/8583c64e-fdea-493d-bdfc-fcfca55992ff" />   
 <br>
 <i>Image showing the final prototype sensor rig. Where: (a) ArduCam CSI camera inside a GoPro waterproof housing, (b) central rig strut, (c) 3D printed camera-housing-to-strut connector, (d) 3D printed strut-to-sensor-box connector, (e) 3D printed torch-to-strut connector, (f) dive torch, (g) epoxy-resin filled sensor housing containing (h) pressure sensor, (not visible) IMU, Jetson Nano and buck converter.</i>
</p>

<br>
<p> ROS (Robot Operating System) is used on the Jetson Nano for data collection between the peripherals - an IMU, pressure sensor and two CSI cameras. ROS Melodic was chosen due to the Jetson Nano Nvidia Jetpack OS being based on Ubuntu 18.04. The Xsens IMU is supported with maintained ROS drivers. The CSI cameras use an edited version of the camera driver from the Nvidia ros_deep_learning GitHub repository. A custom ROS driver is written for the UART pressure sensor (ezo-prs), which was provided to the supplier company for their further use. </p>

<p> Data rates of the collected rosbag data on the Jetson Nano were initially lower than expected for the cameras. The CPU, GPU and RAM usage was monitored during operation, showing the RAM reaching capacity. A reduction in image size from 1280x720 to 600x600 pixels and frames-per-second from 15 to 10 fps resulted in expected data rates and no RAM overflow. </p>

<br>
<p align="center">
 <img width="60%" alt="ROS_development" src="https://github.com/user-attachments/assets/e5e8ab37-084c-4c2b-9c5d-1963f4df7593" />
 <br>   
 <i>Image showing development of the ROS workspace and nodes on the sensor rig, using jtop to monitor CPU, GPU and RAM usage.</i>
</p>

<br>
<p> GoPro Hero 10 waterproof housings are used for the camera housings, as they are low-cost and sized efficiently for the ArduCam CSI M12 lens cameras. The stereo camera baseline was chosen empirically based on a desired image overlap of 80% at a distance of 2 m from the surface of focus while underwater. </p>

<p> The data collection process is controlled and monitored via an Ethernet cable between a host laptop and the Jetson Nano. We use SSH X11 forwarding over Putty between a Windows host computer and the Jetson to view the camera streams with ROS rqt_gui. </p>

<br>
<p align="center">
 <img width="60%" alt="rig_electrical_diagram" src="https://github.com/user-attachments/assets/ea3806b2-2c6c-4d69-9654-9fa6b12294c0" />  
 <br>
 <i>Diagram showing the electrical setup.</i>
</p>


