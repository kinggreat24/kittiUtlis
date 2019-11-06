##  PointCloud Stiching

### Intriduction

A kitti odometry dataset helper package that read data from files and publish them to ROS environments. The topic include: 

- image_left**[sensor_msgs/Image]**
- image_right**[sensor_msgs/Image]**
- odom_gt**[nav_msgs/Odometry]**
- path_gt**[nav_msgs/Path]**
- current_pointcloud**[sensor_msgs/PointCloud]**

### Install

#### requirements

- Ubuntu16.04

- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

  ```shell
  #Setup your sources.list
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  #Set up your keys
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  #Installation
  sudo apt-get update
  sudo apt-get install ros-kinetic-desktop-full
  # Initialize rosdep
  sudo rosdep init
  rosdep update
  # Environment setup
  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  #Dependencies for building packages
  sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
  ```

- OpenCV 3.2

- PCL1.8

#### build

```shell
cd <your_catkin_ws>
catkin_make
```

### RUN

```shell
roslaunch pointcloud_stiching pointcloud_stiching.launch
```



