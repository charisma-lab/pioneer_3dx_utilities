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
sudo dpkg -i libaria_2.9.4+ubuntu12_i386.deb
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
