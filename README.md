# Livox-Horizon-LOAM
## LiDAR Odemetry and Mapping (LOAM) package for Livox Horizon LiDAR
![image](https://github.com/Livox-SDK/livox_horizon_loam/blob/master/rviz_cfg/fig/fig-1.png)
**livox_horizon_loam** is a robust, low drift, and real time odometry and mapping package for [*Livox LiDARs*](https://www.livoxtech.com/), significant low cost and high performance LiDARs that are designed for massive industrials uses. Our package is **mainly designed for low-speed scenes(~5km/h)** and address many key issues: feature extraction and selection in a very limited FOV, and motion distortion compensation. We use [*Ceres-Solver*](http://ceres-solver.org/) for scan matching to avoid complicated differential geometry derivation. The codes are well structured and streamlined to improve readability and extendability.

In the development of our package, we reference to LOAM, [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).


**Developer:** [Livox](https://www.livoxtech.com)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 1.4. **Eigen**
Recommend version [3.3.7](http://eigen.tuxfamily.org/index.php?title=Main_Page).


## 2. Build
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/Livox-SDK/livox_horizon_loam.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 3. Directly run
Connect to your PC to Livox LiDAR (horizon) by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    roslaunch livox_ros_driver livox_lidar_msg.launch
    roslaunch loam_horizon loam_livox_horizon.launch
    
```
If you want to use horizon's internal IMU to eliminate rotation distortion, run
```
    roslaunch livox_ros_driver livox_lidar_msg.launch
    roslaunch loam_horizon loam_livox_horizon_imu.launch
```

## 4. Rosbag Example
### 4.1. **Common rosbag**
Download [parking lot rosbag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/demo/2020_parking_lot.bag) or [outdoor scene rosbag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/demo/2020_open_road.bag) and then
```
    roslaunch loam_horizon loam_livox_horizon.launch
    rosbag play YOUR_DOWNLOADED.bag
```
If you want to use horizon's internal IMU to eliminate rotation distortion, run
```
    roslaunch loam_horizon loam_livox_horizon_imu.launch
    rosbag play YOUR_DOWNLOADED.bag
```


### 4.2. **External IMU rosbag**
If you want to use an external IMU with horizon to eliminate rotation distortion, you need to manually obtain external parameters. For example, we record a rosbag containing external imu data and horizon data, and the extrinsic quaternion is (0, 1, 0, 0), then:
1. Download [external imu rosbag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/demo/imu-demo.bag).
2. Modify the line [25](https://github.com/Livox-SDK/livox_horizon_loam/blob/df742c4aa1f7ea505d3f4ef4aac14b0776f5750a/launch/loam_livox_horizon_imu.launch#L25) of file loam_livox_horizon_imu.launch as follows:
```
     <node pkg="loam_horizon" type="imu_process" name="imu_process" output="screen" >
        <!--<remap from="/imu" to="/livox/imu"/>-->
     </node>
```
3. Modify the [Extrinsic Quaternion](https://github.com/Livox-SDK/livox_horizon_loam/blob/df742c4aa1f7ea505d3f4ef4aac14b0776f5750a/src/imu_processor/data_process.cpp#L23) between horizon and IMU, IMU is the reference:
```
    Eigen::Quaterniond q(0, 1, 0, 0);
```
4. Run
```
    roslaunch loam_horizon loam_livox_horizon_imu.launch
    rosbag play imu-demo.bag
```

## 5.Acknowledgments
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).


