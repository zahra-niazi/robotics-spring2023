// COUNTS_PER_REVOLUTION   36

// WHEEL_RADIUS    0.1524   Wheel radius in meters
// // the distance of a wheel move forward when encoder increased by 1
// DISTANCE_PER_COUNT      ((TWOPI * WHEEL_RADIUS) / COUNTS_PER_REVOLUTION)

// WHEEL_SEPARATION      0.3    two wheels center-to-center distance


// Called from the velocity callback.
// Set the values for the left and right wheels' speeds
// so that Eddie can do arcs
void EddieController::moveLinearAngular(float linear, float angular) {

  // Calculate wheel velocities and convert meter per second to position per second
  double left_command_speed  = ((linear - angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS) / DISTANCE_PER_COUNT;
  double right_command_speed = ((linear + angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS) / DISTANCE_PER_COUNT;

  sem_wait(&mutex_interrupt_);
  left_drive_speed  = left_command_speed;
  right_drive_speed = right_command_speed;
  // this is not a pure rotation
  rotate_ = false;
  process_ = true;
  // cancel other moving commands in favor of this new one.
  interrupt_ = true;
  sem_post(&mutex_interrupt_);
}
