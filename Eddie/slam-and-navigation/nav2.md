# Nav2

Navigation in robotics refers to the ability of a robot to move from one location to another in an environment while avoiding obstacles and reaching its destination safely. The Nav2 project leverages ROS2 for building its navigation stack, which includes various components to enable mobile robot navigation.

## **Action Servers**

In the context of ROS2, action servers are used to manage long-running tasks that may take a significant amount of time to complete, such as navigation. They allow clients to **request** specific tasks, and the server provides feedback and results during the execution of the task.&#x20;

The Nav2 stack utilizes action servers extensively to handle navigation tasks, enabling efficient execution of complex actions like _path planning_ and _control_.

## **Lifecycle Nodes and Bond**

ROS2 introduces the concept of lifecycle nodes, which are nodes that follow a state machine-based lifecycle.&#x20;

This helps in ensuring **deterministic** behavior during the startup and shutdown of ROS2 servers. The bond connection in Nav2 is used to ensure the active status of servers after they transition up. If a server crashes, the bond notifies the lifecycle manager, preventing critical failures.

## **Behavior Trees**

Behavior trees are a way of **organizing** complex robotics tasks in a hierarchical manner. In Nav2, the BehaviorTree CPP V3 library is used to construct behavior trees.&#x20;

These trees consist of various **nodes** representing specific **behaviors or tasks**. Behavior trees provide a formal structure for navigation logic and allow for the creation of complex systems while maintaining verifiability and validation.

## **Navigation Servers**

The Nav2 project employs several action servers to handle different aspects of navigation:

* **Planner Server:** Responsible for computing a **valid path** from the robot’s current position to a goal location based on the global environmental representation.
* **Controller Server:** Handles **local control efforts** to _follow the global plan_ or _execute specific local tasks_, like docking or avoiding obstacles.
* **SmootherServer:** Refines the path computed by the planner to improve its smoothness and overall quality.
* **Behavior Server:** Executes various **recovery behaviors** to deal with unknown or failure conditions, making the system more fault-tolerant.

## **Waypoint Following**

Waypoint following is a fundamental feature of navigation systems. The Nav2 stack includes a waypoint following program with a plugin interface for executing specific tasks at multiple waypoints. It is useful for completing tasks like taking pictures, picking up objects, or waiting for user input at specified locations.

## **State Estimation**

State estimation involves determining the robot’s **pose** (position and orientation) **relative to a global reference frame**.

In Nav2, two main transformations are essential: _`map->odom`_ and _`odom->base_link`_.&#x20;

Global positioning systems (like GPS or SLAM) provide the _`map->odom`_ transformation, while the odometry system (wheel encoders, IMUs, etc.) offers the _`odom->base_link`_ transformation.

## **Environmental Representation**

The environmental representation is how a robot **perceives** and models its surroundings. In Nav2, _**costmaps**_ are used for this purpose.&#x20;

A costmap is a regular 2D grid that assigns costs to cells representing different types of areas (_unknown_, _free_, _occupied_, or _inflated cost_).&#x20;

Various costmap layers, implemented as pluginlib plugins, **buffer** information from sensors into the costmap to provide a comprehensive representation of the environment.

## **Costmap Filters**

Costmap filters in Nav2 are used to apply **spatial-dependent behavioral changes** based on annotations provided in filter masks.&#x20;

Filter masks contain data about **specific areas** in the environment where certain behaviors or restrictions should be applied. Costmap filters read this data and update the underlying costmap to alter the robot’s behavior in those areas.  


<br>
<br>
Overall, Nav2 provides a powerful and flexible framework for mobile robot navigation within the ROS2 ecosystem, with support for various navigation-related concepts and components.  
<br>

<hr style="border:2px solid gray">

<br>
We specifically focus on optimizing the `nav2_velocity_smoother` for our task. In order to do that, we will need to understand the key parameters that influence the smoothing behavior.

The `nav2_velocity_smoother` is a lifecycle-component node that is part of the Nav2 navigation stack. Its main purpose is to take in velocity commands from Nav2’s controller server and apply smoothing to these commands before sending them to the robot’s hardware controllers.&#x20;

It achieves this by taking input commands from the `cmd_vel` topic and producing a smoothed output on the `smoothed_cmd_vel` topic.

Key features and design choices of the `nav2_velocity_smoother` node:

### **Lifecycle and Composition Management.**&#x20;

The node utilizes the ROS2 lifecycle manager for state management, which ensures **predictable behavior** during startup, shutdown, and runtime. Additionally, it utilizes composition for process management, allowing it to work seamlessly with other components in the Nav2 stack.

### **Timer-based Smoothing**

Instead of simply computing a smoothed velocity command in the callback of each `cmd_vel` input from Nav2, the node operates on a regular timer running at a configurable rate.&#x20;

This allows it to interpolate commands at a higher frequency than Nav2’s local trajectory planners can provide. By running at a higher frequency, it provides a more regular stream of commands to the robot’s hardware controllers and performs finer interpolation between the current velocity and the desired velocity. This results in smoother acceleration and motion profiles, leading to better overall performance.

### **Open and Closed Loop Operation Modes**

The node supports two primary operation modes:

* _**Open-loop:**_ In this mode, the node assumes that the robot was able to achieve the velocity sent to it in the last command, which has been smoothed. This assumption is valid when acceleration limits are set properly. It is useful when **robot odometry** is not particularly accurate or has significant latency relative to the smoothing frequency. In open-loop mode, there is no delay in the feedback loop.
* _**Closed-loop:**_ In this mode, the node reads from the odometry topic and applies a smoother over it to obtain the robot’s current speed. This current speed is then used to determine the robot’s achievable velocity targets, taking into account velocity, acceleration, and deadband constraints using live data.

The `nav2_velocity_smoother` node plays a crucial role in enhancing the performance of robot motion control.&#x20;

By smoothing the velocity commands, it reduces jerky movements and wear-and-tear on robot motors and hardware controllers. The ability to operate in different modes allows flexibility in adapting to various robot setups and odometry accuracy levels. Overall, this node is a valuable component in the Nav2 navigation stack, contributing to the smooth and efficient navigation of mobile robots.

The `nav2_velocity_smoother` is responsible for smoothing velocities to reduce wear-and-tear on robot motors and hardware controllers by mitigating jerky movements resulting from some local trajectory planners’ control efforts.&#x20;

The main parameters that we adjust on are:

```yaml
’smoothing_frequency’: 0.1
```

_This parameter controls how quickly the velocity is adjusted to smooth out accelerations and jerky movements. A lower smoothing frequency will result in slower adjustments, while a higher smoothing frequency will make the smoothing more responsive._

```yaml
’scale_velocities’: True
```

_When set to true, this parameter adjusts other components of velocity proportionally to a component’s required changes due to acceleration limits. It tries to make all components of velocity follow the same direction, but still enforces acceleration limits to guarantee compliance, even if it means deviating off the commanded trajectory slightly. This can help in maintaining a smoother and more coordinated motion of the robot._

```yaml
’feedback’: ’OPEN_LOOP’
```

```yaml
’max_velocity’: [1.4, 0.0, 0.2]
```

```yaml
’min_velocity’: [-1.4, 0.0, -0.2]
```

_In some cases, you may want to set a minimum velocity to ensure the robot continues moving even in low-speed situations. This can be useful for preventing the robot from coming to a complete stop during trajectory planning._

```yaml
’max_accel’: [1.125, 0.0, 1.125]
```

_This parameter limits the rate at which the linear and angular velocities can change. By constraining the accelerations, you can prevent sudden and abrupt changes in velocity._

```yaml
 ’max_decel’: [-1.125, 0.0, -1.125]
```

```yaml
’deadband_velocity’: [0.0, 0.0, 0.0]
```

_The deadband is a small range around zero velocity where no adjustments are made. This parameter sets the minimum velocities (m/s) to send to the robot hardware controllers to prevent small commands from damaging hardware controllers if that speed cannot be achieved due to stall torque. It helps avoid applying very small velocities that might lead to wearand-tear on the robot hardware._
