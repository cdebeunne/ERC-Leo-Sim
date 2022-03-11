# ERC Leo Sim

This repository is a fork of https://github.com/EuropeanRoverChallenge/ERC-Remote-Navigation-Sim that enables a Cave world (loaded from https://github.com/LTU-RAI/gazebo_cave_world) and several configurations of LEO rover.

## Prerequisites

The simulation requires ROS (Robot Operating System) and was mainly developed and tested with these distributions:
 - [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/) on [Ubuntu 18.04 Bionic Beaver](https://releases.ubuntu.com/18.04/)
 - [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/20.04/)

Other distributions might not work properly so it is recommended to use one of these two setups. \
The guide will refer to the distribution of your choice (for example, `melodic` or `noetic`) as `<distro>`.

The rest of the tools used in this guide can be installed with apt:
```sh
# if using ROS Melodic
sudo apt install python-rosdep python-catkin-tools
# if using ROS Noetic
sudo apt install python3-rosdep python3-catkin-tools
```

## Building
You need to simply clone this repository, then use rosdep to install any missing dependencies. If you are running `rosdep` for the first time, you might have to run:
```sh
sudo rosdep init
```
first. Then, to install the dependencies, type:
```sh
rosdep update
sudo apt update
rosdep install --rosdistro <distro> --from-paths src -iy
```
Now, use the `catkin` tool to build the workspace:
```sh
catkin_make
```

## Launching

Make sure you source the devel space on each terminal session you want to use the simulation on:
```
source devel/setup.bash
```

To start the simulation and gazebo GUI, type:
```
roslaunch leo_erc_gazebo leo_marsyard.launch
```

You can also try the Cave environnement configuration with:
```
roslaunch leo_erc_gazebo leo_cave.launch
```

To visualize the model in Rviz, type on another terminal session:
```
roslaunch leo_erc_viz rviz.launch
```

Turn on the `Image` panel in Rviz to show the simulated camera images.

To test teleoperation with a keyboard, you can run the `key_teleop` node:
```
rosrun leo_erc_teleop key_teleop
```

To control the Rover using a joystick, type:
```
roslaunch leo_erc_teleop joy_teleop.launch
```

The command mapping was set for the Xbox 360 controller and looks like this:
| Xbox 360 controller       | Command                           |
|---------------------------|-----------------------------------|
| RB button                 | enable - hold it to send commands |
| Left joystick Up/Down     | linear velocity                   |
| Right Joystick Left/Right | angular velocity                  |

To modify it, you can edit the `joy_mapping.yaml` file inside the `leo_erc_teleop` package.

## ROS API

This section describes ROS topics, services and parameters that are available on both the simulation and the real robot.

### Subscribed topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Rover.  
    Only linear.x (m/s) and angular.z (r/s) are used.

* **`zed2/reset_odometry`** ([std_msgs/Empty])

    Resets the odometry published on the `zed2/odom` topic.

* **`probe_deployment_unit/drop`** ([std_msgs/Empty])

    Spawn probe model.

### Published topics

* **`wheel_odom`** ([geometry_msgs/TwistStamped])

    Current linear and angular velocities of the robot estimated from wheel velocities.

* **`wheel_pose`** ([geometry_msgs/PoseStamped])

    A 2D pose of the robot estimated from the wheel velocities.

* **`joint_states`** ([sensor_msgs/JointState])

    Current state of the wheel joints.

* **`camera/image_raw`** ([sensor_msgs/Image])

    Unrectified images from the hazard avoidance camera.

* **`camera/image_raw/compressed`** ([sensor_msgs/CompressedImage])

    JPEG-compressed images from the hazard avoidance camera.

* **`camera/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the hazard avoidance camera (see [image_pipeline/CameraInfo]).

* **`zed2/left_raw/image_raw_color`** ([sensor_msgs/Image])

    Unrectified color images from the left ZED2 camera.

* **`zed2/left_raw/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the left ZED2 unrectified camera.

* **`zed2/left/image_rect_color`** ([sensor_msgs/Image])

    Rectified color images from the left ZED2 camera.

* **`zed2/left/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the left ZED2 camera.

* **`zed2/right_raw/image_raw_color`** ([sensor_msgs/Image])

    Unrectified color images from the right ZED2 camera.

* **`zed2/right_raw/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the right ZED2 unrectified camera.

* **`zed2/right/image_rect_color`** ([sensor_msgs/Image])

    Rectified color images from the right ZED2 camera.

* **`zed2/right/camera_info`** ([sensor_msgs/CameraInfo])

    Calibration data for the right ZED2 camera.

* **`zed2/depth/depth_registered`** ([sensor_msgs/Image])

    Depth map image registered on left ZED2 camera image.

* **`zed2/depth/camera_info`** ([sensor_msgs/CameraInfo])

    Depth camera calibration data.

* **`zed2/point_cloud/cloud_registered`** ([sensor_msgs/PointCloud2])

    Registered color point cloud.

* **`zed2/imu/data`** ([sensor_msgs/Imu])

    Accelerometer, gyroscope, and orientation data from the ZED2 IMU.

* **`zed2/odom`** ([nav_msgs/Odometry])

    Estimated ZED2 Camera position and orientation in free space relative to the Odometry frame (visual-inertial odometry).

* **`ground_truth`** ([nav_msgs/Odometry]) **(Only in the simulation)**

    The actual position of the robot on the terrain. It may be useful for validating performance of a global localization solution.

* **`probe_deployment_unit/probes_dropped`** ([std_msgs/UInt8])

    The actual number of probes dropped. 

### Services

* **`core2/reset_odometry`** ([std_srvs/Trigger])

    Resets the pose published on the `wheel_pose` topic.

* **`probe_deployment_unit/home`** ([std_srvs/Trigger])

    Resets the PDU. In the simulation, it removes dropped probe models and resets the counter.
    **Warning:** Don't use this on the real robot during the competition.

### Parameters set

* **`robot_description`** (type: `str`)

    The URDF model of the robot.

* **`probe_description`** (type: `str`) **(Only in the simulation)**

    The URDF model of the probe.

* **`pdu_node/probe_spawn_translation/x | y | z`** (type: `float`) **(Only in the simulation)**

    Probe spawn point translation from the base_footprint frame.

[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[geometry_msgs/TwistStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
[geometry_msgs/PoseStamped]: http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
[sensor_msgs/JointState]: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/CompressedImage]: http://docs.ros.org/api/sensor_msgs/html/msg/CompressedImage.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
[image_pipeline/CameraInfo]: http://wiki.ros.org/image_pipeline/CameraInfo
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
[nav_msgs/Odometry]: http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
[std_msgs/Empty]: http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html
[std_msgs/UInt8]: http://docs.ros.org/en/melodic/api/std_msgs/html/msg/UInt8.html
[std_srvs/Trigger]: http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html

### Troubleshooting

* If there is a problem with the spawner, a gazebo server may be still running. You can fix this by running:
```
killall -9 gzclient
killall -9 gzserver
```

