# underwaterSLAM_2D_lowCost

This is still under Construction.

A low cost Underwater SLAM system, which operates with the help of a DVL, an IMU, and a mechanical rotating Sonar.
It works with the help of the Fourier SOFT 2D(FS2D) registration, which can be looked up here:

In general, this package provides an experimental approach, which still has much room for improvement.
See this as a start for further research/Development.

You can test different settings with the help of [ekfRobot.yaml](config%2FekfRobot.yaml).
Start SLAM + EKF first,(with [experiment1_launch.py](launch%2Fexperiment1_launch.py)), then start the ROS2 Bag



# Packages necessary:

* GTSAM `https://github.com/borglab/gtsam.git` `git checkout 4.2a7` We used `-DGTSAM_USE_SYSTEM_EIGEN=ON` in the compiling setting
* OpenCV 4.7 `https://github.com/opencv/opencv.git`
* FFTW3 `sudo apt install libfftw3-dev`
* message definitions for the SLAM usage `https://github.com/constructor-robotics/datasetMessageDefinitions.git`
* CGAL `sudo apt install libcgal-dev`
All the other Libraries are either standard(Eigen, ROS2, OpenGT etc.), or inside this package.(FMS registration(uses library [soft20](https://github.com/artivis/soft20.git), Peak
detection(have a look at find-peaks))
An Example Bag in `data`, where the SLAM can be performed on is at included in this package. Look in CMakeLists.txt for other
libraries if something is missing.




