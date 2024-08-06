# FinalProjectYaronBay

my project files and explenations

## Requirements


1. This project is was made in Ubuntu 22.04 OS.

2. use Ros2 humble distribution installation in the link below:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

3. install intel realsense sdk and ros wrapper as described in the link below:

https://github.com/IntelRealSense/realsense-ros


4. ensure to have these python packages:

open cv - `pip3 install opencv-python`

ulralytics (YOLO) - `pip install ultralytics`

scikit-learn - `pip install scikit-learn`

open 3d - `pip install open3d `

serial - `pip install pyserial`



## Activation


1. initialize the depth camera and set pointcloud:
    ```
    ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true 
    
    ros2 param set /camera/camera pointcloud.enable true
    ```


2. run the camera motor driver

3. run the yolo depth node

