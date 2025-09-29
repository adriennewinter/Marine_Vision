# Marine Vision

## MSc (Research) Mechatronics Engineering dissertation.

Author: Adrienne Winter. 
Supervisors: Robyn Verrinder, Professor Edward Boje and Doctor Paul Amayo. 
Research Group: African Robotics Unit (ARU) at the University of Cape Town (UCT).
Funders: South African International Maritime Institute (SAIMI) and Oppenheimer Memorial Trust. 

This dissertation explores the use of underwater SLAM (Simultaneous Localization and Mapping) for visual reconstruction of ship hulls — a problem with major implications for the maritime industry, which carries the majority of global trade. The research focuses on camera-based methods for mapping in feature-sparse underwater environments in two main areas - dataset creation and visual odometry in feature-sparse underwater scenes. 

### Sensor Rig Construction
A lack of well-documented underwater v-SLAM datasets was noted and a prototype multi-sensor rig was designed and tested for the creation of a SLAM dataset (stereo-inertial-pressure). The rig construction highlighted practical lessons in hardware robustness. 

The ROS workspace and 3D printing files are provided here. 

### Underwater Camera Calibration
The logistical difficulty of collecting good underwater camera calibration, necessary for producing high-quality SLAM datasets, was noted. An open-source calibration method that addresses this (the Pinax camera calibration model) was evaluated and contributed to. The Pinax calibration method was shown to significantly improve underwater stereo depth estimation accuracy (6.2% mean error using the Pinax method versus 49.4% with traditional calibration).

### Underwater SLAM Dataset Creation
An underwater SLAM dataset including stereo images, inertial (IMU) and pressure readings was created in a controlled tank at the Institute for Marine Technology (IMT) in Cape Town, South Africa and made publicly available. A C++ ROS data synchronization package was created and made publicly available that synchronizes up to 4 sensor topics using timestamps and preserves original sensor frequencies.

### Stereo Visual Odometry Pipeline
A stereo visual odometry (VO) pipeline was developed and evaluated for feature-sparse underwater scenes, comparing SIFT and ORB features with adaptive thresholding and image enhancement methods. Results showed ORB with image de-hazing, CLAHE filtering and robust outlier rejection delivered the most stable and efficient performance in underwater scenes.



