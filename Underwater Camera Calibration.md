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
