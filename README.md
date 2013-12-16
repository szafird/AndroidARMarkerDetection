This repository contains source code for a Java-based augmented reality marker detection application for Android. The code is largely based on the Aruco library. It depends on OpenCV4Android and a pre-setup xml configuration file containing camera intrinsics that should match the format of the OpenCV camera calibration program. Currently, the code expects the calibration file path to be "/calibration/camera.xml"

To use:

1. Add the OpenCV4Android library to the project (project properties -> android -> add library)
2. Set the project as an android library
3. Initialize a MarkerDetector
4. Pass each frame to the MarkerDetector
