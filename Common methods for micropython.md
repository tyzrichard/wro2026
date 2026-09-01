# Pybricks EV3 MicroPython: Common Methods for Motors and Sensors

## Motor Movement Methods

- `run(speed)`  
  Runs the motor at a constant speed (degrees per second).

- `run_time(speed, time)`  
  Runs the motor at a speed for a set time (milliseconds).

- `run_angle(speed, angle)`  
  Rotates the motor by a specific angle (degrees) at a set speed.

- `run_target(speed, target_angle)`  
  Moves the motor to a specific absolute angle at a set speed.

- `stop()`  
  Stops the motor using the default stop mode.

- `hold()`  
  Holds the motor at its current position.

- `brake()`  
  Brakes the motor (stops quickly, but does not hold position).

- `reset_angle(angle)`  
  Sets the current angle to a specific value (usually 0).

- `angle()`  
  Returns the current angle of the motor (degrees).

---

## Sensor Methods

### TouchSensor

- `pressed()`  
  Returns `True` if the sensor is pressed.

### ColorSensor

- `color()`  
  Returns the detected color.

- `reflection()`  
  Returns the reflection (light intensity).

- `ambient()`  
  Returns the ambient light level.

- `rgb()`  
  Returns the raw RGB values.

### UltrasonicSensor

- `distance()`  
  Returns the distance to an object (millimeters).

- `presence()`  
  Returns `True` if an object is detected.

### GyroSensor

- `angle()`  
  Returns the current angle (degrees).

- `reset_angle(angle)`  
  Sets the current angle to a specific value.

- `speed()`  
  Returns the rate of rotation (degrees per second).

### InfraredSensor

- `distance()`  
  Returns the distance to an object (proximity).

- `buttons(channel)`  
  Returns the buttons pressed on the remote (if used).