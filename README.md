# ISP2021-visual_slam
This is the github project for the F1Tenth Independent Study Projects 2021. In this project we want to develop a VISUAL SLAM algorithm that is capable creating a map based on camera data as well as localize the car based on camera data.


## Software Requirements
- Linux Ubuntu (tested on versions 18.04)
- ROS Melodic.
- Python 3.69.

## Hardware Requiremenrts
- Realsense D435i
- Nvidia Jetson TX2

## Dependencies
- Librealsense2
- Kimera
- Segmentation Pipeline

## Installation
- Clone the current repository in a new workspace.
- Install Librealsense2 on your system follow the instruction listed [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).
- Install Librealsense2 on the Jetson along with the kernel updates of the latest version use the instructions listed [here]((https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)).
- Setup and install [Kimera VIO ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS) and the [Kimera - Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)
- Install the [Segmentation Pipeline](https://github.com/Ravi3191/Seg_F1) in your current workspace. 


## Running the code
* ```roscore``` 
* ```roslaunch realsense2_camera rs_camera.launch unite_imu_method:=linear_interpolation```
* `Step 3:` ....


