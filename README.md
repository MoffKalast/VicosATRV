# VicosATRV


### source -> all packages in the robot ROS workspace

### img -> WIP images

### CAD -> stl models designed for 3D printing

## Initialization

The motor driver part of the robot can be turned on using the following tutorial:

https://trac.vicos.si/ros/wiki/ATRVMiniSetup

Next up, press the green button on the rear of the robot (or the one inside) to turn on the computer.

After that it's set up to connect to ViCoS_5Ghz under the following static IP:

SSH:

    ssh atrv@192.168.33.106

The username and password are set to 'atrv' and 'visionm4n'.


## Shutdown

If it's running first turn of the PC by either running something like:

    sudo poweroff

or holding the red button inside the robot beside the green one if a wifi/ssh connection cannot be established.

Then turn of the motor driver part by accessing the menu on the rear display of the robot, entering the [PWR] submenu and then pressing [Kill PWR].

## Pan Tilt

The used pan tilt ROS driver is http://wiki.ros.org/flir_ptu_driver, with the docs on the linked wiki page.

## Head LEDs




## Realsense D415

The Realsense depth camera requires the Realsense SDK https://github.com/IntelRealSense/librealsense and the ROS driver https://github.com/IntelRealSense/realsense-ros which are both installed on the robot.

It can be launched using one of the launch files in the realsense2_camera package.


## Speakers

The speakers are 

Speaker volume can be adjusted using the following command:

    sudo amixer cset numid=1 80%

## Speakers



## Dodatni viri

https://github.com/vicoslab/atrv_mini_ros


