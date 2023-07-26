# 2D SLAM

## slam\_toolbox

The `slam_toolbox` package incorporates information from laser scanners in the form of a Laser-Scan message and TF transforms from `odom->base_link`, and creates a map 2D map of a space.&#x20;

This package will allow you to fully serialize the data and pose-graph of the SLAM map to be reloaded to continue mapping, localize, merge, or otherwise manipulate. SLAM Toolbox can be run in **synchronous** (process all valid sensor measurements, regardless of lag) and **asynchronous** (process valid sensors measurements on an as-possible basis) modes.

`slam_toolbox` provides various tools and capabilities to address the problems with SLAM, and it offers several features:

1. **Basic 2D SLAM for Mobile Robotics:** `slam_toolbox` allows users to perform standard point-and-shoot 2D SLAM, where a mobile robot explores the environment, builds a map, and saves it in PGM (Portable Graymap) file format. The library also includes utilities to facilitate map saving.
2. **Continuing and Refining Mapping:** slam\_toolbox allows us to continue mapping from a saved pose-graph. This means a previously serialized map can be loaded, and the robot can continue exploring and refining the map.
3. **Life-Long Mapping:** The library supports life-long mapping, where a robot can load a previously saved pose-graph and continue mapping in a space while intelligently removing extraneous information from newly added sensor scans.
4. **Optimization-Based Localization:** The library provides an optimization-based localization mode that utilizes the pose-graph. It allows the robot to determine its pose accurately based on the map and sensor data. Additionally, it offers the option to run localization mode without a prior map, using "lidar odometry" mode with local loop closures for localization.
5. **Synchronous and Asynchronous Modes:** slam\_toolbox offers both synchronous and asynchronous modes for mapping, giving flexibility in how data is processed and utilized.


slam\_toolbox’s precess consists of four important steps:

### ROS Node:&#x20;

SLAM toolbox is run in synchronous mode, which generates a ROS node. This **node** subscribes to **laser scan** and **odometry** topics, and publishes _`map->odom`_ transform and a **map**.

### Get odometry and LIDAR data:&#x20;

A callback for the laser topic will generate a **pose** (using odometry) and a **laser scan** tied at that node. These _**PosedScan**_ objects form a queue, which are processed by the algorithm.

### Process Data:&#x20;

The queue of **PosedScan** objects are used to construct a _**pose graph**_; odometry is refined using laser scan matching.&#x20;

This pose graph is used to **compute robot pose**, and **find loop closures**. If a loop closure is found, the pose graph is optimized, and pose estimates are updated.&#x20;

Pose estimates are used to compute and publish a _`map->odom`_ transform for the robot.

### Mapping:&#x20;

Laser scans associated with each pose in the pose graph are used to construct and publish a map.


[Here](https://youtu.be/gvVIIlhwI-I) is our attempt at testing slam\_toolbox alongside nav2.

<p align = "center">
<img src = "../assets/MAP.png">
</p>
<p align = "center">
The map we can obtain in the video
</p>
