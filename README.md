# Marine Vision

## MSc (Research) Mechatronics Engineering dissertation.

**_Author:_** Adrienne Winter.  
**_Supervisors:_** Robyn Verrinder, Professor Edward Boje and Doctor Paul Amayo.  
**_Research Group:_** African Robotics Unit (ARU) at the University of Cape Town (UCT).  
**_Funders:_** South African International Maritime Institute (SAIMI) and Oppenheimer Memorial Trust.  

This dissertation explores the use of underwater SLAM (Simultaneous Localization and Mapping) for visual reconstruction of ship hulls — a problem with major implications for the maritime industry, which carries the majority of global trade. The research focuses on camera-based methods for mapping in feature-sparse underwater environments in two main areas - dataset creation and visual odometry in the context of SLAM. 

<p align="center" width="100%">
  <img height="100" alt="aru_logo_white_background" src="https://github.com/user-attachments/assets/b710eaf6-498c-4cb0-98bb-21ffc8ffa6c5" />
  <img height="100" alt="UCT_logo_white_background" src="https://github.com/user-attachments/assets/907ebc2f-bec3-45a7-ba6a-42625b5bee29" />
  <img height="100" alt="Oppenheimer_logo_square" src="https://github.com/user-attachments/assets/21219b9b-c347-4302-a237-6f47f215a86b" />
  <img height="100" alt="SAIMI_logo_square" src="https://github.com/user-attachments/assets/5f01d86c-2608-4fb0-9a49-4b7946678e85" />
</p>

## Sensor Rig Construction
A lack of well-documented underwater SLAM datasets was noted and a prototype multi-sensor rig was designed and tested for the creation of a dataset (stereo-inertial-pressure). The rig construction highlighted practical lessons in hardware robustness. 

The ROS workspace and 3D printing files are provided [here](https://github.com/adriennewinter/Marine_Vision/tree/main/Sensor%20Rig%20Construction). 

<p align="center" width="100%"> 
  <img width="40%" alt="final_rig" src="https://github.com/user-attachments/assets/2588dbc9-776d-476c-81fb-c9ec3b47b700" /> 
  <br>
  <i>Image of the prototype sensor rig.</i>
</p>

## ROS Data Synchronisation Package
A C++ ROS data synchronisation package was created that synchronises up to 4 sensor topics using timestamps and preserves original sensor frequencies.

The package can be found in the ROS workspace of the sensor rig [here](https://github.com/adriennewinter/Marine_Vision/tree/main/Sensor%20Rig%20Construction/ROS_Workspace/src/synchronize).

<p align="center" width="100%"> 
  <img width="70%" alt="Synch_Depth_Problem" src="https://github.com/user-attachments/assets/43214f55-888e-4c97-bbda-4dc386f07a85" />
  <br>
  <i>Diagram of a feature depth estimation issue encountered when synchronisation is performed using timestamps created by different clocks.</i>
</p>

## Underwater Camera Calibration
The logistical difficulty of collecting good underwater camera calibration, necessary for producing high-quality SLAM datasets, was noted. An open-source calibration method that addresses this (the Pinax camera calibration model) was evaluated and contributed to. The Pinax calibration method was shown to significantly improve underwater stereo depth estimation accuracy (6.2% mean error using the Pinax method versus 49.4% with traditional calibration).

The details of underwater camera calibration are provided [here](https://github.com/adriennewinter/Marine_Vision/blob/main/Underwater%20Camera%20Calibration.md).

<p align="center" width="100%"> 
  <img width="40%" alt="depth_maps_with_orig_pics" src="https://github.com/user-attachments/assets/337ef96c-3f54-4af4-a261-23c6ee8ed850" />
  <img width="26%" alt="Pinax_model_projection-4_arrows" src="https://github.com/user-attachments/assets/69570c90-3ae7-42f1-8da7-fcb44dff98be" />
  <br>
  <i>(left) Depth maps used to evaluate the Pinax calibration method's performance. (right) Light refraction projection removal using the Pinax model.</i>
</p>

## Underwater SLAM Dataset Creation
An underwater SLAM dataset including stereo images, inertial (IMU) and pressure readings was created in a controlled tank at the Institute for Marine Technology (IMT) in Cape Town, South Africa and made open source available [here](https://github.com/African-Robotics-Unit/Ship-Hull-Vinyl-Dataset). 

<p align="center" width="100%"> 
  <img width="25%" alt="rig_on_tank_platform" src="https://github.com/user-attachments/assets/871a5691-49c1-45a5-8eac-7dc4595ab01f" /> 
  <img width="48.5%" alt="VIO_pressure_stereo_Factor_Graph" src="https://github.com/user-attachments/assets/a1ded966-d38f-43f1-9e0e-7f7dbc98ca8e" />
  <br>
  <i>(left) Sensor rig in front of the tank used for data collection at IMT. (right) Visual example of a pressure-stereo-inertial factor graph.</i>
</p>  

## Stereo Visual Odometry Pipeline
A stereo visual odometry (VO) pipeline was developed and evaluated for feature-sparse underwater scenes, comparing SIFT and ORB features with adaptive thresholding and image enhancement methods. Results showed ORB with image de-hazing, CLAHE filtering and robust outlier rejection delivered the most stable and efficient performance in underwater scenes.

Details of the VO pipeline are provided [here](https://github.com/adriennewinter/Marine_Vision/blob/main/Visual%20Odometry%20Pipeline.md).

<p align="center" width="100%"> 
  <img width="60%" alt="VO_SLAM_Context" src="https://github.com/user-attachments/assets/852f1bcf-ba88-4370-beea-8f66ebd028e5" /> 
  <br>
  <i>Diagram showing where visual odometry sits in the larger context of underwater SLAM for reconstructions.</i>
</p>
