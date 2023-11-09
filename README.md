# pioneer_3dx_utilities

## System Requirements
ubuntu 20.04

ros neotic


## RosAria install

RosAria acts as a bridge between ros and the pioneers.  The pioneers use a esoteric serial communication protocol.  RosAria handles this is the background so you can just prioneer away. 

### RosAria Documentation

Installation guide the following is based on can be found [here](https://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)

Overview of what RosAria offers us can be found [here](https://wiki.ros.org/ROSARIA)

### Instalation Guide

In your catkin workspace, see 0.1 [here](https://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA) if you don't have one. Perform the following:

```console
cd ~/catkin_ws/src
git clone https://github.com/amor-ros-pkg/rosaria.git
```

Next, we will diverage from the above instructions and install the following:

```console
sudo apt-get install libaria-dev
```

Then `catkin_make` in your workspace directory.

At this point you can attempt to run the basic RosAria node as follows:

```console
rosrun rosaria RosAria
```

Note that you will need a roscore node running and to have the pioneer's USB plugged in. If you get errors relating to permissions see system setup below.

### System Setup
Running the pioneer's from a USB port requires some small permission changes. For each user on the computer you will need to perform the following:

```console
sudo usermod -a -G tty username
sudo usermod -a -G dialout username
```

You will need to log out and back in for the changes to take effect. This will allow ros to user your USB ports for RosAria. If your still not connecting it could be because your looking at the wrong port address.  To find the address, unplug the USB and run the following command:

```console
ls /dev/tty*
```

Then plug back in your Pioneer and run the same command a second time.  There will be a new address that appears this is the address for your pioneer. Try the following command with to make sure the address works, you need to replace ttyS0 with your specific address:

```console
rosrun rosaria RosAria _port:=/dev/ttyS0
```    
## PS4 Controller
Verify first that the "joy" package is  installed in your ROS system. It should already be there as it is part of the desktop install but to confirm run the following

```console
sudo apt-get install ros-noetic-joy
```
Now in your catkin workspace src folder run the following:

```console
git clone https://github.com/solbach/ps4-ros.git
```

The instructions on its use are [here](https://github.com/solbach/ps4-ros) and the readme. After following those to get the PS4 controller working, you then need to launch the following node:

```console
rosrun pioneer_3dx_utilities ps4_controller_node.py
```

Now have fun children.


## RP Lidar Install

To get data from the lidar sensors  you will need the RP Lidar package. There is a known bug in distribution 2.0.0, so we will have to checkout the 1.7.0 tag.

```console
git clone https://github.com/Slamtec/rplidar_ros.git
git checkout tags/1.7.0
```

To test, use the following launch and then view it in Rviz

```console
roslaunch pioneer_3dx_utilities pioneer_sensors.launch
```


## Navigation

Navigation is a complex process and so I would recommend understanding [this](https://wiki.ros.org/nav_core). The basic idea is that there is a global planner who creates a global trajectory.  They give small parts of the global trajectory to the local planner. The local planner uses sensor data to ensure it is on a collision free pathway.  What is complicated is localization and mapping. For basic planning you need to install the following pacakges:

```console
sudo apt-get install ros-noetic-move-base*
sudo apt-get install ros-noetic-amcl*
```

The first package is the basic move-base pacakge that is linked to above. This gives your ros system a bunch of different msg types and stuff. Several configuration files are used to set up the global and local parameters full described [here](https://wiki.ros.org/navigation/Tutorials/RobotSetup). The following files configure the navigation stack:
- config/costmap_common_params.yaml
- config/base_local_planner_params.yaml
- config/local_costmap_params.yaml
- config/global_costmap_params.yaml
These files are currently configured to reasonable values. The full navigation pipeline can be started with the following command:

```console
roslaunch pioneer_3dx_utilities nav.launch
```

This will load the robot discription, start the lidar, create the appropriate frames for the odometry and sensing. It will then launch amcl (see below) and set up the basic move_base pipeline. You have no reason to change these files for any reason. Create a new repo, or create your own configuration files and nav.launch file.

The second package is amcl it stands for [Adaptive Monte Carlo Localization](https://wiki.ros.org/amcl). This is a localization algorithm. You will give it a lidar topic and odometry information and it will tell you the most likely position of the robot in the map. It also requires a map, this can be generated offline using [this](https://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData) method, or can be done online with gmapping (SLAM), as shown below.

### SLAM
TBD






