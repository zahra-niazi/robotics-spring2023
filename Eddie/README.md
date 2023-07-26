# Exploration of SLAM and Navigation with Eddie

This is a report on the exciting exploration of 2D and Visual SLAM and navigation with Eddie, a versatile differential drive robot.

In this report, we delve into a detailed analysis of Eddie's hardware and software components, including its firmware command set. We also discuss the challenges we encountered while setting up Eddie's Kinect sensor and the step-by-step process of bringing Eddie to life and teleoperating it from a static laptop.

Then, we unravel Eddie's kinematic model and delve into the mathematical intricacies involved in obtaining the odometry for our differential drive robot. We present a thorough analysis of the formulas and calculations used to precisely track Eddie's movements.

Next, we deep dive into 2D SLAM and ROS2 navigation techniques, testing the methods to create a map of Eddie's environment and exploring how Eddie successfully navigates through it.

Lastly, we explore various visual SLAM libraries. Ultimately we decide on RTABMap, and we evaluate its performance with Eddie.




# Table of contents

* [Introduction](README.md)

## Eddie setup and bringup

* [Hardware Subsystems](eddie-setup-and-bringup/hardware-subsystems.md)
* [Control Board Firmware Command Set](eddie-setup-and-bringup/control-board-firmware-command-set.md)
* [Software Subsystems](eddie-setup-and-bringup/software-subsystems.md)
* [Setting up the Kinect](eddie-setup-and-bringup/setting-up-the-kinect.md)
* [Bringup](eddie-setup-and-bringup/bringup.md)
* [Networking](eddie-setup-and-bringup/networking.md)

## Odometry

* [Motor Control](odometry/motor-control.md)
* [Wheel Odometry Model](odometry/wheel-odometry-model.md)

## SLAM and Navigation

* [2D SLAM](slam-and-navigation/2d-slam.md)
* [Nav2](slam-and-navigation/nav2.md)
* [vSLAM](slam-and-navigation/vslam.md)
