# Lidar Camera Calibration

This repository implements extrinsic calibration of the Mid360 LiDAR and Realsense D435i camera on an Nvidia Jetson device using the livox_camra_calib library. The original repositories involved are as follows:

* livox_camera_calib: [https://github.com/hku-mars/livox_camera_calib#](https://github.com/hku-mars/livox_camera_calib#)
* livox_ros_driver2: [https://github.com/Livox-SDK/livox_ros_driver2.git](https://github.com/Livox-SDK/livox_ros_driver2.git)
* realsense_ros: [https://github.com/IntelRealSense/realsense-ros.git](https://github.com/IntelRealSense/realsense-ros.git)

Due to the need for some modifications to compile on the Arm architecture, the source code in this repository has been modified to ensure immediate use. This ReadMe file will primarily inform you of the modified source code.

----

# 1. Confirming the Hardware and Software

We conducted experiments on the following hardware and software devices. If your hardware and software environment differs from ours, we recommend modifying the ReadMe file according to your version and replacing the sections relevant to your version.

[Note]: If your device is a Jetson series device and uses Noetic, there is little need to change anything.

|Device|OS|ROS|
|---|----|---|
|Nvidia Jetson Orin DK|Ubuntu 20.04|Noetic|

Support for more devices will be added after we complete testing.

----

# 2. Install Dependencies

According to the requirements of the [livox_camera_calib](https://github.com/hku-mars/livox_camera_calib) library, you need to install the following dependencies:

## 2.1 ROS Dependencies
```bash
$ sudo apt-get install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-pcl-conversions
```
## 2.2 Other Dependencies
You can refer to the installation links provided for other dependencies, but I found that installing Eigen and PCL directly with the following command works without any problems. Only Ceres requires a source code installation:

```bash
$ sudo apt-get install libeigen3-dev libpcl-dev
```

* Eigen: [http://eigen.tuxfamily.org/index.php?title=Main_Page](http://eigen.tuxfamily.org/index.php?title=Main_Page);
* Ceres: [http://ceres-solver.org/installation.html](http://ceres-solver.org/installation.html);
* PCL: [http://www.pointclouds.org/downloads/linux.html](http://www.pointclouds.org/downloads/linux.html)

---

# 3. Pull the source code and compile it

Because the Jetson platform comes with an OpenCV library installed during the firmware update, sometimes we may need to compile our own OpenCV library to run SLAM. Installing ROS also installs a ROS-specific version of OpenCV, making the entire environment quite confusing. Installing third-party libraries and dependencies is quite tedious.

## 3.1 Pull the livox_camera_calib source code
Create a workspace called `calib_ws` and pull the source code:

```bash
$ mkdir -p calib_ws/src
$ cd calib_ws/src
$ git clone https://github.com/hku-mars/livox_camera_calib.git
```
## 3.2 Pull the cv_bridge source code
Return to the `calib_ws` workspace and pull the source code. However, don't rush to put it in the `src` directory. We only need the `cv_bridge` portion of this repository:

```bash
$ cd calib_ws
$ git clone https://github.com/ros-perception/vision_opencv.git
$ cd vision_opencv
$ git checkout noetic # Switch branches based on your ROS version
```

After switching to the correct branch, Copy the `cv_bridge` folder to the `calib_ws/src` directory.

```bash
$ cd calib_ws
$ cp -r calib_ws/vision_opencv/cv_bridge calib_ws/src
```

## 3.3 Modifying Source Code
This requires modifying multiple source code locations. The order of modification can vary. For information on compiling the CUDA-accelerated OpenCV library on Jetson, please refer to the corresponding content in my flashing blog post (note that you need to compile version 4.2.0) or the `"Step 3. Compiling OpenCV 4.2.0"` section in the [ReadMe](https://github.com/GaohaoZhou-ops/JetsonSLAM/blob/main/fast_livo2_project/resources/ReadMe-CN.md) Github repository. However, please note: **After compiling, do not use `sudo make`. Install**.

Experimentally, I found that <font color=red>**OpenCV 4.5.4**</font> does not work, but <font color=green>**OpenCV 4.2.0**</font> works.

* `src/livox_camera_calib/CMakeLists.txt`:

```cmake
# 1. Set the OpenCV 4.2.0 directory

set(OpenCV_DIR "/home/orin/Desktop/JetsonSLAM/third_party/opencv-4.2.0/build")
# 2. Add the CUDA toolkit

find_package(CUDAToolkit REQUIRED)

find_package(PCL REQUIRED)
# 3. Specify the OpenCV version

find_package(OpenCV 4.2.0 REQUIRED) # find_package(OpenCV)

find_package(Threads)
# 4. Specify the Ceres version

find_package(Ceres 2.2 REQUIRED) # find_package(Ceres REQUIRED)
```

* `src/cv_bridge/CMakeLists.txt`:
```cmake
set(OpenCV_DIR "/home/orin/Desktop/JetsonSLAM/third_party/opencv-4.2.0/build")
find_package(OpenCV 4.2.0 QUIET)
# set(_opencv_version 4)
# find_package(OpenCV 4 QUIET)
```

* `src/livox_camera_calib/src/lidar_camera_calib.cpp`
```cpp
// ceres::LocalParameterization *q_parameterization =
// new ceres::EigenQuaternionParameterization();
ceres::Manifold *q_parameterization = new ceres::EigenQuaternionManifold();
```

* `src/livox_camera_calib/src/lidar_camera_multi_calib.cpp`
```cpp
// ceres::LocalParameterization *q_parameterization =
// new ceres::EigenQuaternionParameterization();
ceres::Manifold *q_parameterization = new ceres::EigenQuaternionManifold();
```

## 3.4 Compiling the Project
After modifying the source code above, you can compile the entire project. If you have already activated the conda environment and have not made any additional configuration, it is recommended to exit the conda environment first.

```bash
$ conda deactivate
$ cd calib_ws
$ catkin_make
```

![build_success](./images/build_success.png)

If you do not follow the above steps, especially those related to OpenCV, you may see the following warning. Do not ignore this warning, or the program will crash upon startup:



```bash
/usr/bin/ld: warning: libopencv_features2d.so.4.5, needed by /home/orin/opencv-4.5.4/build/lib/libopencv_calib3d.so.4.5.4, may conflict with libopencv_features2d.so.4.2
/usr/bin/ld: warning: libopencv_imgproc.so.4.2, needed by /home/orin/Desktop/calib_ws/libopencv_features2d.so.4.2, may conflict with libopencv_imgproc.so.4.5
/usr/bin/ld: warning: libopencv_core.so.4.2, needed by /home/orin/Desktop/calib_ws/libopencv_features2d.so.4.2, may conflict with libopencv_core.so.4.5
[100%] Built target lidar_camera_calib
/usr/bin/ld: warning: libopencv_features2d.so.4.5, needed by /home/orin/opencv-4.5.4/build/lib/libopencv_calib3d.so.4.5.4, may conflict with libopencv_features2d.so.4.2
/usr/bin/ld: warning: libopencv_imgproc.so.4.2, needed by /home/orin/Desktop/calib_ws/libopencv_features2d.so.4.2, may conflict with libopencv_imgproc.so.4.5
/usr/bin/ld: warning: libopencv_core.so.4.2, needed by /home/orin/Desktop/calib_ws/libopencv_features2d.so.4.2, may conflict with libopencv_core.so.4.5
```

![build_failed](./images/build_failed.png)

----

# 4. Project Configuration
This section also requires some source code modifications, but does not involve compiling. It's recommended that you first run the official demo before implementing your own requirements. Related resource links are as follows:

* Official demo data: [https://pan.baidu.com/s/1oz3unqsmDnFvBExY5fiBJQ?pwd=i964](https://pan.baidu.com/s/1oz3unqsmDnFvBExY5fiBJQ?pwd=i964)

## 4.1 Obtaining Camera Intrinsic Parameters
Camera intrinsic parameters will be frequently used during the calibration process, and obtaining them is a very basic operation. Two methods are provided here.

### 4.1.1 Direct Acquisition
If your device is a RealSense D435i or another mass-produced series, the easiest way is to view the camera intrinsic parameters through the driver. For information on installing the RealSense SDK and ROS project, please refer to my flashing blog post, which I will not elaborate on here.

Note: Since the `realsense-ros` repository is not pulled in the current workspace, I assume you have already pulled it in another workspace named `realsense_ws` and completed the compilation.

```bash
$ cd realsense_ws
$ source devel/setup.bash
$ roslaunch realsense2_camera rs_d435_camera_with_model.launch
```

![camera_rviz](./images/camera_rviz.png)

Then view the contents of the topic `/camera/color/camera_info`. The information in the topic means the following:

* `K`: Camera intrinsic parameter matrix camera_matrix (what we need);
* `D`: Distortion coefficients dist_coeffs;
* `R`: Rotation matrix rotate_matrix;
* `P`: Projection matrix project_matrix;

```bash
rostopic echo /camera/color/camera_info

height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [607.2879028320312, 0.0, 322.6806640625, 0.0, 606.75830078125, 250.453369140625, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [607.2879028320312, 0.0, 322.6806640625, 0.0, 0.0, 606.75830078125, 250.453369140625, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi:
x_offset: 0
y_offset: 0
height: 0
width: 0
do_rectify: False
```

### 4.1.2 Internal Calibration
If your camera does not support direct acquisition, you will need to use the calibration tool. First, install the dependencies:

```bash
$ sudo apt-get install ros-$ROS_DISTRO-camera-calibration
```

Then start the calibration tool and prepare a checkerboard calibration board. You can generate one directly from the following website:

* [https://markhedleyjones.com/projects/calibration-checkerboard-collection](https://markhedleyjones.com/projects/calibration-checkerboard-collection)

Here we still use Taking the Realsense D435i device as an example, the following command parameters have the following meanings:

* `--size 8x6`: The number of checkerboard corner points. Counting directly indicates 9 black and white squares horizontally and 7 black and white squares vertically;
* `--square`: The size of a single checkerboard square, in meters;
* `image`: The topic name to which the image will be published;

```bash
$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/camera/color/image_raw
```

Move the camera to view the checkerboard from different angles and distances until the `CALIBRATE` button turns green, then click Start Calculation:

* `X`: Left and right movement;
* `Y`: Up and down movement;
* `Size`: Forward and backward movement;
* Skew: Tilt and rotate;

![camera_inner_calib](./images/camera_inner_calib.png)

Calculating the internal parameters may take a while. Once completed, you should see the following output in the terminal:

```bash
Camera matrix
608.970206 0.000000 309.853058
0.000000 604.838795 257.667687
0.000000 0.000000 1.000000

Distortion
0.145507 -0.252373 0.006365 -0.011785 0.000000

Rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
621.799255 0.000000 302.949580 0.000000
0.000000 621.586121 259.744721 0.000000
0.000000 0.000000 1.000000 0.000000
```

## 4.2 Single Sensor Pair - Demo
Modify the configuration file `src/livox_camera_calib/config/calib.yaml` and replace the paths and camera parameter matrices with your own. If you are running the demo, you do not need to modify the camera parameter matrix:

```yaml
# Data path. adjust them!
common: 
image_file: "/home/orin/Desktop/calib_ws/dataset/single_scene_calibration/0.png" 
pcd_file: "/home/orin/Desktop/calib_ws/dataset/single_scene_calibration/0.pcd" 
result_file: "/home/orin/Desktop/calib_ws/src/livox_camera_calib/result/extrinsic.txt"

# Camera Parameters. Adjust them!
camera: 
camera_matrix: [1364.45, 0.0, 958.327, 
0.0, 1366.46, 535.074, 
0.0, 0.0, 1.0 ] 
dist_coeffs: [0.0958277, -0.198233, -0.000147133, -0.000430056, 0.000000]

# Calibration Parameters.!
calib:
calib_config_file: "/home/orin/Desktop/calib_ws/src/livox_camera_calib/config/config_outdoor.yaml"
use_rough_calib: true # set true if your initial_extrinsic is bad

Run the following program in the terminal. The time required will vary depending on the hardware configuration, but it is normal for the rviz screen to be completely black upon startup because feature and residual calculations have not yet begun:

```bash
$ cd calib_ws
$ source devel/setup.bash
$ roslaunch livox_camera_calib calib.launch
```

The program will pop up five windows in sequence, and in the final rviz window, The feature matching results are displayed in the command:

![feature_extract](./images/feature_extract.png)

After the calculation is complete, the following output will be displayed in the terminal. This calculation is very resource-intensive and takes about 5 minutes. The `push enter to publish again` prompt will appear, indicating that the calibration is complete:

```bash
q_dis:0.00483663, t_dis:0.00348196
Iteration:39 Dis:11 pnp size:12172
iter cost cost_change |gradient| |step| tr_ratio tr_radius ls_iter iter_time total_time
0 5.418889e+04 0.00e+00 2.20e+04 0.00e+00 0.00e+00 1.00e+04 0 1.99e+00 2.00e+00
1 5.415701e+04 3.19e+01 1.96e+01 0.00e+00 1.00e+00 3.00e+04 1 2.09e+00 4.09e+00
Ceres Solver Report: Iterations: 2, Initial cost: 5.418889e+04, Final cost: 5.415701e+04, Termination: CONVERGENCE
q_dis: 0.00837903 , t_dis: 0.00304098
Push enter to publish again

```

![calib_result](./images/calib_result.png)

The calibration results will be saved to `src/livox_camera_calib/result/extrinsic.txt` In the file:
```txt
-0.00265775,-0.999901,-0.0138502,0.0146354
-0.00333279,0.013859,-0.999898,0.0573609
0.999991,-0.00261132,-0.00336929,-0.0519612
0,0,0,1
```

## 4.3 Single Sensor Pair - Real Device
This project requires a rosbag data package and an image captured by an RGB camera. When capturing images, please note the following:

1. The radar and camera must not be moving during acquisition;
2. The captured object must be a stationary object;
3. The larger the scale, the better.
4. Ideally, the radar and camera should be facing the object.

### 4.3.1 Starting the Livox Mid360 LiDAR

Assuming you have compiled the Livox Mid360 LiDAR workspace named `livox_ws` and configured the LiDAR and local IP addresses in `livox_ws/src/livox_ros_driver2/config/MID360_config.json`, you can obtain the LiDAR's IP address from the device box or by looking at the last two digits of the serial number on the back of the LiDAR. For example, if the serial number is 74, the address is `192.168.1.174`. If none of these conditions are met, you can only view or modify the address using Livox-Viewer:

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.110",		// Local IP address
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.110",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.110",
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.110",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.174",				// lidar IP address
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

After entering the workspace, start the LiDAR. Once started, you can find the LiDAR topic in the rviz interface and set the parameter `Deacy Time` to `1` to display a 1s cumulative point cloud. Because Mid360 uses a non-repeating scan, viewing only the real-time point cloud is not very intuitive.

```bash
$ cd livox_ws
$ source devel/setup.bash
$ roslaunch livox_ros_driver2 rviz_MID360.launch
```
### 4.3.2 [Optional] Limiting the Point Cloud Range
Since Mid360 is 360 degrees horizontally, and the calibration tool solves a point-narrow-point problem, it's best to reduce the amount of data during calculations for better performance. We provide a node script that subscribes to the raw point cloud topic and only retains point clouds within a certain angle.

[Note]: When recording data packets, you need to replace the topic name;

