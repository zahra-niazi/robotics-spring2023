# Setting up the Kinect

The `kinect_ros2` package is a component that allows the Eddie robot to interface with a Microsoft Kinect sensor and utilize its RGB and depth images for perception tasks in ROS2.&#x20;

This package spins a node responsible for receiving the RGB and depth images from the Kinect sensor and storing them in memory. Then, at regular intervals, it publishes the data from these images on specific ROS2 topics. The topics it publishes are:

* <mark style="color:green;">`image_raw`</mark>: This topic contains the raw RGB image data captured by the Kinect sensor. It provides a continuous stream of color images representing what the camera perceives from its viewpoint.
* <mark style="color:green;">`camera_info`</mark>: This topic carries calibration and intrinsic parameters of the camera. It includes information about the camera’s focal length, optical centers, and distortion coefficients. This data is essential for performing accurate transformations and geometric calculations in image processing.
* <mark style="color:green;">`depth/image_raw`</mark>: Here, the package publishes the raw depth image data captured by the Kinect sensor. The depth image provides information about the distance of objects from the camera. It is represented as a 2D array of depth values corresponding to each pixel in the RGB image.
* <mark style="color:green;">`depth/camera_info`</mark>: Similar to the <mark style="color:green;">`camera_info`</mark> topic, this topic contains calibration and intrinsic parameters specific to the depth camera. These parameters enable accurate depth mapping and 3D reconstruction from the depth image.

By publishing these topics, the `kinect_ros2` package enables other nodes in the ROS2 environment to subscribe to and access the Kinect sensor’s data for various perception and navigation tasks.


<p align = "center">
<img src = "../assets/Screenshot from 2023-07-02 16-21-17.png">
</p>
<p align = "center">
Testing Kinect
</p>


One of the issues we encountered was the presence of out-of-sync timestamps between the <mark style="color:green;">`camera_info`</mark> and <mark style="color:green;">`depth/camera_info`</mark> topics that were being published. These two topics provide important calibration and intrinsic parameters for the RGB camera and the depth camera, respectively.&#x20;

Having these parameters synchronized correctly is crucial for accurate perception and 3D reconstruction. When performing coordinate transformations between RGB and depth images or other perception tasks, using unsynchronized timestamps can lead to misalignments and inaccurate results.&#x20;

This affects the quality of visual data and hinders the performance of perception algorithms. Also in applications like mapping and navigation, incorrect timestamps can introduce errors in robot localization and obstacle avoidance. These errors can potentially lead to collisions or inefficient path planning. Accurate timestamps are crucial for correctly associating corresponding data from different sensors or perception modules. With out-of-sync timestamps, the associations can become incorrect, leading to flawed data interpretations.

To address this issue, we fixed the out-of-sync timestamps. We ensured that both topics publish their data with matching timestamps, ensuring that the calibration and intrinsic parameters correspond accurately to the corresponding RGB and depth images.&#x20;

By resolving the timestamp synchronization problem, we improved the quality and reliability of our perception and mapping processes. It allowed our robot to better perceive and interpret the environment, leading to more accurate navigation and decision-making capabilities. As a result, the overall performance and robustness of our robot’s perception system were greatly enhanced, allowing it to operate more effectively and efficiently in its environment.
