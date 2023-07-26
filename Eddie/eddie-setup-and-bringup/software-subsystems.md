# Software Subsystems

## Robot Communication: **$\textcolor{Plum}{\texttt{eddiebot\\_bringup}}$**

The communication with the board involves establishing a connection between the robot control board and our ROS2 programs, enabling seamless data exchange. This software acts as a pathway that conveys and receives data between two of our major hardware components. Specifically, this sub-system serves as the means to convey the desired velocity information from the control system to be executed by the robot’s firmware board.&#x20;

## Description: $\textcolor{Plum}{\texttt{eddiebot\\_description}}$

This package contains files and data used to describe the structure of eddie. It includes URDF files to define the physical properties and kinematic structure of eddie. This package enables accurate simulation of Eddie’s structure and movement within the ROS environment. This is crucial for visualizing and controlling the robot in a virtual space before deploying it in the physical world.

The `diff_drive` plugin is a component added to the URDF files to simulate the differential drive mechanism that eddie uses.

The `joint_state_publisher` plugin is another component added to the URDF files. It is used to simulate the movement of Eddie’s wheels. `joint_state_publisher` helps in updating the positions and orientations of robot joints based on the data received from the robot’s actuators.

In the launch file we run two nodes:

* `robot_state_publisher`: The robot\_state\_publisher reads the URDF description file and generates static transformations (tf static transforms). These transformations define the spatial relationship between different components of the robot, such as its base, wheels and different sensors.
* `joint_state_publisher`: The joint\_state\_publisher node sends dynamic transformations from the wheels to the robot\_state\_publisher. These dynamic transformations update the joint positions based on the movement of the wheels, allowing the `robot_state_publisher` to accurately track and represent the robot’s current state.

## Simulation: $\textcolor{Plum}{\texttt{eddiebot\\_gazebo}}$

The eddiebot\_gazebo package enables us to test and experiment with Eddie’s behavior in the Gazebo simulation environment.

When we run the main launch file in this package, it sets up the Gazebo simulator, creating a simulated environment where Eddie can exist and interact. Once Gazebo is up, we also spin a node to "spawn" Eddie inside the simulated world. Meaning we place the robot model, represented by a URDF file, into the Gazebo environment. We do this by launching the `eddiebot_description` launch file mentioned before.

This step essentially brings Eddie to life in the simulation, and we can now control and observe its actions as if it were a real physical robot. To ensure communication between Gazebo and the ROS2 ecosystem, the eddiebot\_gazebo package establishes a connection between their respective topics.

## Custom Interfaces: $\textcolor{Plum}{\texttt{eddiebot\\_msgs}}$

In the eddiebot\_msgs package, we define the custom interfaces for Eddie, the robot. This package allows us to define the specific topics and services through which we can communicate information about Eddie’s current state and control its movements.

Firstly, we define the topics where information about the robot’s current state will be published. This includes data such as its current speed, angle, and sensor inputs. By publishing this information on specific topics, other nodes in the ROS2 ecosystem can subscribe to these topics and receive real-time updates on Eddie’s state. This enables different components of the system to access and utilize this information for various purposes, such as perceiving the environment, making decisions or performing actions based on Eddie’s current state.

Additionally, we define services that can be used to communicate with the robot and instruct it to move or turn. Services in ROS2 provide a request-response mechanism, allowing nodes to send requests to specific services and receive corresponding responses. For Eddie, these services can be used to send commands to the robot, such as specifying a desired movement or rotation. The robot can then receive these commands, process them, and respond accordingly, enabling control over Eddie’s actions through the defined services.

By defining these custom interfaces in the eddiebot\_msgs package, we establish a standardized and consistent way to exchange information and control commands between different components of the system, facilitating effective communication and interaction with Eddie.

## Odometry: $\textcolor{Plum}{\texttt{eddiebot\\_odom}}$

The eddiebot\_odom package plays a crucial role in enabling Eddie to understand its position and movement within the environment. It is responsible for publishing odometry information, which is an estimation of Eddie’s position and orientation based on the motion of its wheels. Odometry is essential for mapping and localization tasks in robotics.

Since Eddie doesn’t have an IMU (Inertial Measurement Unit), which could provide direct information about its orientation and acceleration, the package relies on wheel encoders. Wheel encoders are sensors mounted on the robot’s wheels that measure the rotation of the wheels. By analyzing the data from these encoders, the eddiebot\_odom node can calculate how far each wheel has turned and, consequently, estimate Eddie’s overall movement.

The process starts with the eddiebot\_odom node subscribing to the topic where the wheel encoder data is published. As the wheels move, the encoders send real-time information to the node about the rotations. Using this information, the node calculates the incremental movement of Eddie in terms of distance and angular change.

To keep track of Eddie’s pose (position and orientation) over time, the node continuously updates the transformation between two frames: the <mark style="color:green;">`/odom`</mark> frame and the <mark style="color:green;">`/base_footprint`</mark> frame.

* The <mark style="color:green;">`/odom`</mark> frame represents Eddie’s initial position in the world, usually set at the robot’s starting point. The coordinate frame called odom is a world-fixed frame. The pose of a mobile platform in the odom frame can drift over time, without any bounds. This drift makes the odom frame useless as a long-term global reference. However, the pose of a robot in the odom frame is guaranteed to be continuous, meaning that the pose of a mobile platform in the odom frame always evolves in a smooth way, without discrete jumps. The odom frame is useful as an accurate, short-term local reference, but drift makes it a poor frame for long-term reference.
* The <mark style="color:green;">`/base_footprint`</mark> frame is attached to Eddie’s base chassis. The coordinate frame called `base_link` is rigidly attached to the mobile robot base. The base\_link can be attached to the base in any arbitrary position or orientation; for every hardware platform there will be a different place on the base that provides an obvious point of reference.

By knowing how far Eddie has moved from its initial position and the angular changes during its motion, the node can estimate the transformation between these frames. This allows Eddie to understand its current pose relative to where it started, effectively providing odometry information.

Having accurate odometry enables Eddie to navigate its environment more effectively. It can use this information to create a map of the environment. However, it’s worth noting that odometry estimates may drift over time due to inaccuracies and wheel slippage, especially in complex environments. To improve localization accuracy, other sensor modalities like visual odometry or an IMU can be used in conjunction with wheel encoders.

## Visualization: $\textcolor{Plum}{\texttt{eddiebot\\_rviz}}$

The eddiebot\_rviz package is responsible for visualizing Eddie’s current state and movement in a visualization environment like RViz.

We run the `eddiebot_description` launch file within the eddiebot\_rviz package, which loads the robot’s URDF model into RViz. This URDF model includes information about Eddie’s physical structure, joints, sensors, and other components, allowing RViz to create a visual representation of the robot accurately.

The eddiebot\_rviz package subscribes to various topics that publish real-time data about Eddie’s state and movements. It subscribes to topics that provide information about joint angles and sensor data. With access to the subscribed topics, the eddiebot\_rviz package can update the visualization of Eddie’s state in real-time within the RViz environment. As the robot moves, the URDF model’s joints and links are updated according to the received data, enabling you to see Eddie’s current pose and joint angles dynamically.

The real-time visualization in RViz allows us to interactively analyze Eddie’s behavior. We can observe how the robot responds to different commands and paths, assess its trajectory, and verify whether its movements align with the intended behavior. In addition to joint angles and pose, the eddiebot\_rviz package can also display other sensor data within RViz. For example, it can visualize data from range sensors, cameras, or any other sensors equipped on the robot. This feature is valuable for understanding how the robot perceives its environment and how it responds to various stimuli.

## Velocity: $\textcolor{Plum}{\texttt{eddiebot\\_vel\\_controller}}$

The eddiebot\_vel\_controller package subscribes to the <mark style="color:green;">`/cmd_vel`</mark> topic, where commands to move or rotate Eddie are published. These commands are in the Twist format which consists of linear and angular velocity components that define how Eddie should move in the environment.

Upon receiving the Twist messages from the <mark style="color:green;">`/cmd_vel`</mark> topic, the eddiebot\_vel\_controller extracts the linear and angular velocity values from the Twist message and converts them into simple velocity commands. Then it publishes these commands on the <mark style="color:green;">`eddie/simple_velocity`</mark> topic.

## SLAM and Navigation: $\textcolor{Plum}{\texttt{eddiebot\\_nav}}$

The eddiebot\_nav package is the central location for managing all the launch files configuring the navigation stack and SLAM tools used in the Eddie robot’s autonomous navigation capabilities. Eddie uses the slam\_toolbox for 2D SLAM. SLAM is a critical process that allows the robot to create a map of its environment while simultaneously determining its own position within that map. The `slam_toolbox` is a powerful library that performs SLAM algorithms to achieve accurate mapping and localization using sensor data from onboard sensors like LIDAR and odometry. Because Eddie is not equipped with a LIDAR, we launch the `depthimage_to_laserscan` package to extract laser scans.

In addition to 2D SLAM, the eddiebot\_nav package also explores vSLAM using RTAB-Map. RTAB-Map is a popular vSLAM library that enables the robot to construct 3D maps of the environment using both visual and depth information from RGB-D cameras. This advanced vSLAM technique enhances the accuracy and richness of the mapping process, enabling Eddie to navigate more efficiently in complex environments. For autonomous navigation, Eddie utilizes the Nav2 framework. The navigation stack is a collection of algorithms and components responsible for planning and executing the robot’s path from its current location to the target destination. In this launch file we also run the static transformations from the robot’s frame to the RGB and depth frames.

