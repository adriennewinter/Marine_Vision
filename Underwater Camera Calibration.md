# The Pinax Model for Underwater Camera Calibration

## GitHub Repo
The GitHub repo for implementing this method can be found [here](https://github.com/adriennewinter/Pinax-Camera-Model).

We make some changes to the authors’ available MATLAB code to improve ease of use. Our adaptation converts all OpenCV functions in the scripts to equivalent MATLAB native functions, so the scripts are ready to run with no additional integrations between MATLAB and external libraries, which can be difficult and add to development time.

This calibration method is used on the documented underwater SLAM [dataset](https://github.com/African-Robotics-Unit/Ship-Hull-Vinyl-Dataset) we collected in the indoor tank at the Institute For Maritime Technology (IMT) in Simon’s Town, Cape Town.

## Underwater Camera Calibration
Without good camera calibration there are not many useful computer vision tasks that can be achieved. Camera calibrations are used to remove
distortion introduced by camera lenses (intrinsics) and to rectify stereo images (extrinsics), which allows us to make use of epipolar geometry constraints to simplify various computer vision tasks, such as calculating disparity between a stereo pair to estimate scene depth or searching for feature correspondence between frames.

In the case of underwater cameras, light refracts through different mediums – usually water, glass, air – before reaching the image sensor, creating refraction distortions in addition to the regular lens intrinsics. The traditional way of calibrating for these refractive distortions would be to perform a calibration with a checkerboard or AprilTag board using Zhang’s method in the medium of data collection, usually in a pool or in-situ before data collection in the field. This does not sound too difficult, until you consider practical logistics.

In-situ calibration can be logistically difficult when working in the ocean, where visibility is often poor, resulting in sub-optimal calibrations. Calibrating in a controlled pool or tank where visibility is likely better than the ocean is practically easier, but because the refraction index of water has a noticeable impact on the distortion effects, calibrating in water that has a sufficiently different refraction index to the actual data collection will result in sub-optimal image undistortion for computer vision tasks.

A newer approach to underwater camera intrinsic calibration that attempts to solve the issues of logistical difficulty and calibration accuracy is the Pinax model ([paper](https://www.sciencedirect.com/science/article/pii/S0029801817300434)). The authors, Luczynski et al., show that with knowledge of the physical underwater camera housing and refraction indices of the media that light is travelling through, the additional distortion caused by refraction can be removed computationally. The “normal” in-air intrinsic calibration is then used to remove the distortions introduced by the camera lens itself.

The Pinax model works for flat viewport camera housings (as opposed to domed viewports). When manufactured properly, round viewports can convert light refraction so that the camera model underwater behaves like a simple perspective projection camera. But, if the camera axis is slightly off-centre from the centre of the dome, the geometry of the refractions changes and the image distortions no longer behave like a perspective projection camera. Using a flat viewport for underwater camera housings at moderate depths is less expensive, and logistically easier to setup, making underwater vision more accessible. Unfortunately, the flat viewport introduces distortion due to light refraction between changing mediums – water, glass and air.

## The Pinax Model
We provide an explanation of the method here with additional diagrams, contributed by us, that are not found in the [original paper](https://www.sciencedirect.com/science/article/pii/S0029801817300434) for enhanced understanding of the process. 

Agrawal et. al. ([paper](https://ieeexplore.ieee.org/abstract/document/6248073)) make the finding that a camera in an underwater housing with a flat viewport can be modelled as an axial camera. This model differs from the pinhole model because light rays do not intersect the camera optical axis at a single point. Luczynski et al. show in the Pinax model [paper](https://www.sciencedirect.com/science/article/pii/S0029801817300434) that when the camera is placed sufficiently close to the viewport, light rays that are traced from the water medium would theoretically intersect the optical axis along a section that is small enough to approximate as a single point – in the 0.008 mm range. They make use of this fact and the findings by Agrawal et. al. ([paper](https://ieeexplore.ieee.org/abstract/document/6248073)) to define the Pinax model – a mapping between a virtual pinhole camera and the physical underwater axial camera. The benefit of this mapping is that once refraction distortions have been removed numerically, we can use the camera intrinsics found in-air to remove the remaining lens distortions, with no tedious in-water calibrations necessary. The output of the Pinax method is then undistorted underwater images that can be used for further image processing.

The diagram below illustrates the concept of the physical axial camera and virtual pinhole equivalent. Where: nw, ng and na are the water, glass and air refraction indices respectively. d1 is the glass thickness. d0 is the distance of the physical camera center of projection to the viewport. x is the distance of the virtual camera focal point to the outside of the viewport. α is the angle of incidence in the air medium. β and γ are the angle of refraction in the glass and water media respectively. δ is the angle of reflection.

<p align="center" width="100%">
  <img width="40%" alt="Pinax_model_projection-4_arrows" src="https://github.com/user-attachments/assets/ab362c89-8496-456d-a1e7-d3765305b7bd" />
  <br>
  <i>Diagram showing the relationship between the physical axial camera and virtual pinhole equivalent.</i>
</p>

<p align="center" width="100%">
  <img width="40%" alt="Pinax_model_projection-1" src="https://github.com/user-attachments/assets/eb8e75fb-25ce-40a2-af21-26038457d69e" />
  <img width="40%" alt="Pinax_model_projection-6" src="https://github.com/user-attachments/assets/a6938ac6-64ce-4d76-a249-8ccad5e278f7" />
</p>
