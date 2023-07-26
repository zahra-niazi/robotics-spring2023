# Networking

To enable teleoperation of Eddie using two laptops, we need to ensure that both laptops are connected to the same network. This allows them to communicate with each other.&#x20;

In ROS2, the default middleware for communication is DDS (Data Distribution Service). DDS utilizes the concept of Domain ID to enable different logical networks to share a physical network.&#x20;

In ROS2, nodes within the same domain can discover and exchange messages with each other, while nodes in different domains cannot.&#x20;

By default, all ROS2 nodes use domain ID 0. The domain ID is used by DDS to calculate the UDP ports for discovery and communication.&#x20;

When a node starts, it broadcasts its presence to other nodes on the network within the same ROS domain, which is determined by the <mark style="color:red;">`ROS_DOMAIN_ID`</mark> environment variable. Nodes respond with their information, allowing the necessary connections to be established for communication between nodes.



By connecting the laptops in this manner, we can proceed with teleoperating Eddie. We can bring up rviz on the remote laptop, and configure it to display the RGB image from the Kinect sensor. This enables visualization of the camera feed on the remote laptop, providing a convenient way to monitor Eddie’s environment during teleoperation.
