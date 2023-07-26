# Motor Control

As each wheel spins the sensor will gather data about the angular position change of the wheel. Once the encoder input has been captured it must be converted to linear velocity and used by a robotic kinematic model to create an estimate of the distance traveled and possible error. Two wheel encoders are used to deduce the speed and travel direction by measuring the number of pulses registered from each wheel.&#x20;

Each wheel encoder has two sensors and is capable of registering a distance resolution of 1/36th of the robot’s wheel circumference. The Position Controllers on Eddie use a quadrature encoder system to reliably track the position and speed of each wheel at all times. With the included plastic encoder disks, each Position Controller has a resolution of 36 positions per rotation; this equates to approximately 0.5 inches of linear travel per position using the included 6 inch tires.&#x20;

The Position Controllers calculate and report position and average speed data on command. Knowing that the sensor has 1/36th resolution of wheel circumference, the sensor produces 36 pulses for every complete revolution of the wheel. Based on this, the distance traveled in the duration of one pulse is given below:

$$
d=\frac{2\pi r}{36}
$$

```cpp
// COUNTS_PER_REVOLUTION 36
// WHEEL_RADIUS 0.1524 Wheel radius in meters
// // the distance of a wheel move forward when encoder increased by 1
// DISTANCE_PER_COUNT ((TWOPI * WHEEL_RADIUS) / COUNTS_PER_REVOLUTION)

// WHEEL_SEPARATION 0.3 two wheels center-to-center distance

// Called from the velocity callback.
// Set the values for the left and right wheels’ speeds
// so that Eddie can do arcs 
void EddieController::moveLinearAngular(float linear, float angular) {
    // Calculate wheel velocities and convert meter per second to position per second
    double left_command_speed = ((linear - angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS) / DISTANCE_PER_COUNT; 
    double right_command_speed = ((linear + angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS) / DISTANCE_PER_COUNT;
    sem_wait(&mutex_interrupt_);
    left_drive_speed = left_command_speed;
    righ t_drive_speed = right_command_speed;
    // this is not a pure rotation
    rotate_ = false;
    process_ = true;
    // cancel other moving commands in favor of this new one.
    interrupt_ = true;
    sem_post (& mutex_interrupt_ );
}
```
