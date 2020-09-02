# VicosATRV


##### source -> all packages in the robot ROS workspace

##### img -> WIP images

##### CAD -> stl models designed for 3D printing

## Initialization

The motor driver part of the robot can be turned on using the following tutorial:

https://trac.vicos.si/ros/wiki/ATRVMiniSetup

Next up, press the green button on the rear of the robot (or the one inside) to turn on the computer.

After that it's set up to connect to ViCoS_5Ghz under the following static IP:

SSH:

    ssh atrv@192.168.33.251

The username and password are set to 'atrv' and 'visionm4n'.

Once inside you can start all of the currently implemneted nodes by running:

    roslaunch atrv_mini_ros atrv_full.launch 

The functionalities launched will be:

- atrv motor drivers

- flir ptu driver

- realsense2 camera

- led controller

- soundplay node


Test driving the robot can be done using:

    roslaunch atrv_mini_ros teleop.launch

## Shutdown

If it's running first turn of the PC by either running something like:

    sudo poweroff

or holding the green button inside the robot if a wifi/ssh connection cannot be established (acts as a normal shutdown button, the red one is a reboot button).

Then turn of the motor driver part by accessing the menu on the rear display of the robot, entering the [PWR] submenu and then pressing [Kill PWR].

## Pan Tilt

The used pan tilt ROS driver is http://wiki.ros.org/flir_ptu_driver, with the docs on the linked wiki page.

The pan tilt can be controlled through the JointState topic /ptu/cmd, where angles can be set in radians.

There's a helper script that can make it easier to test:

    rosrun flir_ptu_driver cmd_angles 0.9 0.3

The first value being the pan and the second the tilt. Make sure to set the values in a reasonable way to avoid collisions.

## Head LEDs

The package face_led contains a demo script and a topic controlled script that interfaces with the arudino in the head that sets the actual LEDs.

One can run them manually like so:

    rosrun face_led led_controller.py
    python demo.py

The led controller subscribes to two topics:

    /face_emotion
    /face_talk_anim

First one takes a String that sets the overall face disposition our of the following: ["blank", "neutral", "neutral_left", "neutral_right", "furious", "mad", "stupid", "sad", "miserable", "happy", "nauseated", "surprised", "loading"].

The second one takes a Float32 that sets how long the mouth should do a talking animation from the moment the message is received. The animation will be applied to the currently set emotion set by the topic above.


## Realsense D415

The Realsense depth camera requires the Realsense SDK https://github.com/IntelRealSense/librealsense and the ROS driver https://github.com/IntelRealSense/realsense-ros which are both installed on the robot.

They can be launched using one of the launch files in the realsense2_camera package.

Related topics:

```
/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressed/parameter_descriptions
/camera/color/image_raw/compressed/parameter_updates
/camera/color/image_raw/compressedDepth
/camera/color/image_raw/compressedDepth/parameter_descriptions
/camera/color/image_raw/compressedDepth/parameter_updates
/camera/color/image_raw/theora
/camera/color/image_raw/theora/parameter_descriptions
/camera/color/image_raw/theora/parameter_updates
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/depth/image_rect_raw/compressed
/camera/depth/image_rect_raw/compressed/parameter_descriptions
/camera/depth/image_rect_raw/compressed/parameter_updates
/camera/depth/image_rect_raw/compressedDepth
/camera/depth/image_rect_raw/compressedDepth/parameter_descriptions
/camera/depth/image_rect_raw/compressedDepth/parameter_updates
/camera/depth/image_rect_raw/theora
/camera/depth/image_rect_raw/theora/parameter_descriptions
/camera/depth/image_rect_raw/theora/parameter_updates
/camera/extrinsics/depth_to_color
/camera/extrinsics/depth_to_infra1
/camera/extrinsics/depth_to_infra2
/camera/infra1/camera_info
/camera/infra1/image_rect_raw
/camera/infra1/image_rect_raw/compressed
/camera/infra1/image_rect_raw/compressed/parameter_descriptions
/camera/infra1/image_rect_raw/compressed/parameter_updates`
/camera/infra1/image_rect_raw/compressedDepth
/camera/infra1/image_rect_raw/compressedDepth/parameter_descriptions
/camera/infra1/image_rect_raw/compressedDepth/parameter_updates
/camera/infra1/image_rect_raw/theora
/camera/infra1/image_rect_raw/theora/parameter_descriptions
/camera/infra1/image_rect_raw/theora/parameter_updates
/camera/infra2/camera_info
/camera/infra2/image_rect_raw
/camera/infra2/image_rect_raw/compressed
/camera/infra2/image_rect_raw/compressed/parameter_descriptions
/camera/infra2/image_rect_raw/compressed/parameter_updates
/camera/infra2/image_rect_raw/compressedDepth
/camera/infra2/image_rect_raw/compressedDepth/parameter_descriptions
/camera/infra2/image_rect_raw/compressedDepth/parameter_updates
/camera/infra2/image_rect_raw/theora
/camera/infra2/image_rect_raw/theora/parameter_descriptions
/camera/infra2/image_rect_raw/theora/parameter_updates
/camera/realsense2_camera_manager/bond
/camera/rgb_camera/auto_exposure_roi/parameter_descriptions
/camera/rgb_camera/auto_exposure_roi/parameter_updates
/camera/rgb_camera/parameter_descriptions
/camera/rgb_camera/parameter_updates
/camera/stereo_module/auto_exposure_roi/parameter_descriptions
/camera/stereo_module/auto_exposure_roi/parameter_updates
/camera/stereo_module/parameter_descriptions
/camera/stereo_module/parameter_updates
```

## Speakers

The speakers are currently connected through the analog audio jack but an alternative should be found to avoid induction noise.

Speaker volume can be adjusted using the following command:

    sudo amixer cset numid=1 80%

## Microphones

The robot contains two microphones in its 'ears' that are connected to a 'SAMSON Go Mic Connect' stereo microphone.

There's no ROS integration at this time.

## Dodatni viri

https://github.com/vicoslab/atrv_mini_ros

