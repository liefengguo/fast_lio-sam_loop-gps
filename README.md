# fast_lio-sam_loop-gps
**Fast-lio with loop factor and GPS factor for back-end optimization. lio and back-end implementation are moved to "include/core" for better readability.**

## 0. Features
GPS information is integrated into fast_lio-sam_loop-gps to build a consistent map in a large-scale environment. We mainly follow the implementation in [LIO-SAM-6axis](https://github.com/JokerJohn/LIO_SAM_6AXIS) about the system init when we use a 6-axis imu. In addition, we add a two-manual parameter (manual_gps_init+manual_init_yaw) when you are sure about the transformation between the imu and ENU coordinate system. The rebuild ikd-tree modes can still be chosen when the loop closes（correct_fe_en parameter in config file）. 

The system computing efficiency is better than [LIO-SAM-6axis](https://github.com/JokerJohn/LIO_SAM_6AXIS) for incrementally maintaining about ikd-tree map in the odometry. So, using correct_fe_en == false is better for most cases.

![Picture](https://github.com/Hero941215/fast_lio-sam_loop-gps/blob/main/fast_lio-sam_loop-gps.png

## 1. Prerequisites
### 1.0 **gcc** and **g++**

gcc and g++ 7.5.0 are tested OK. 

### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 18.04**

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.3, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### 1.3. **livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

### 1.4. **gtsam**

Use 4.0.0 version in ubuntu 18.04.

### 1.5. **rviz_satellite** (optional)

Follow the "rviz_satellite" bag in [LIO-SAM-6axis](https://github.com/JokerJohn/LIO_SAM_6AXIS) for better visualization. 

## 2. Build

Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/Hero941215/fast_lio-sam_loop-gps
    cd fast_lio-sam_loop-gps
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 1.3 **livox_ros_driver**)

## 3. Run

### 3.1. Download Dataset ([vlp-16](https://github.com/JokerJohn/LIO_SAM_6AXIS)): 

rosbag play XXX.bag

### 3.2. Run SLAM system: 

roslaunch fast_lio_sam_loop mapping_velodyne_gps.launch

## 4. Acknowledgments

Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)，[FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM), [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC), [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), [LIO-SAM-6axis](https://github.com/JokerJohn/LIO_SAM_6AXIS).

