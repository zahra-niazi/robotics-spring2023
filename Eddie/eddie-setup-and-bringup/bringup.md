# Bringup

After successfully setting up the Kinect package, we proceeded to bring up Eddie and test his movement. The steps we followed are as follows:

## Granting USB permissions:&#x20;

We connected the USB cable between Eddie and our laptop and executed the command "sudo chmod a+rw /dev/ttyUSB0" to give the necessary permissions for communication with Eddie’s control board.

```sh
sudo chmod a+rw /dev/ttyUSB0
```

## Bringing up Eddie:&#x20;

To initiate the robot’s functionality, we ran the command "" This command launched the required ROS2 nodes responsible for communicating with Eddie’s control board and enabled communication within the ROS2 environment.

```sh
ros2 launch eddiebot_bringup eddie.launch.yaml
```

## Launching Navigation Stack:&#x20;

To enable navigation capabilities, we executed the following command. This step published static transforms specified in the `eddiebot_odom` package and established transformations between the chassis and the base link of our sensors.

```sh
ros2 launch eddiebot_nav eddiebot.launch.py
```

## Teleoperating Eddie:&#x20;

Finally, to control Eddie’s movement, we ran the following command. This command allowed us to teleoperate Eddie by sending velocity commands to the robot, thus enabling movement in the desired direction.

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

By following these steps, we were able to successfully bring up Eddie, utilize the Kinect package for sensor data, and teleoperate the robot for movement using ROS2 commands. We also bring up RViz to get a better sense of how Eddie is being represented in the ROS2 environment:

```sh
ros2 run eddiebot_rviz view_model.launch.py description:='True'
```

You can see our attempt at teleoperating Eddie [here](https://youtu.be/LtZwPk3DaKk).
