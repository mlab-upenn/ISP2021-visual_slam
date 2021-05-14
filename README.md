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
- Install Librealsense2 on the Jetson along with the kernel updates of the latest version use the instructions listed [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md).
- Setup and install [Kimera VIO ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS) and the [Kimera - Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)
- Install the [Segmentation Pipeline](https://github.com/Ravi3191/Seg_F1) in your current workspace. 


## Running the code for 3D reconstruction of map

* Start ROS Master for communication between nodes \
```roscore``` 

* Launch the Intel RealSense camera node with the following parameters \
```roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true enable_infra1:=true enable_infra2:=true unite_imu_method:=linear_interpolation infra_width:=848 infra_height:=480 infra_fps:=15```

* Disable the camera IR emitter \
```rosrun dynamic_reconfigure dynparam set /camera/stereo_module emitter_enabled 0```

* Source your workspace \
```source ~/devel/setup.bash```

* Launching the Kimera VIO node with the follwing parameters with Loop closure detection (use_lcd) enabled \
 ```roslaunch kimera_vio_ros kimera_vio_ros_realsense_IR.launch run_stereo_dense:=true should_use_sim_time:=false use_lcd:=true```
 
* Launch Kimera Semantics node \
  ```roslaunch kimera_semantics_ros kimera_metric_realsense.launch run_stereo_dense:=true online:=true register_color:=true use_sim_time:=false```
  
* Visualise the Reconstruction on RViz \
```rviz -d $(rospack find kimera_semantics_ros)/rviz/kimera_semantics_euroc.rviz``` 
