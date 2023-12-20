# Terrain Following Node Documentation

## Overview
This Python script implements a terrain-following algorithm for a drone using ROS (Robot Operating System). The drone adjusts its altitude based on inputs from two lidar sensors and maintains a specified threshold above the ground.

## Dependencies
- ROS (Robot Operating System)
- Python 3

## Node Information
- **Node Name:** terrain_follower_node
- **Published Topics:**
  - `/i_drone/terrain_follow` (UInt32): Publishes the throttle PWM signal for terrain following.
- **Subscribed Topics:**
  - `/depth_camera_value` (Float32): Subscribes to the first lidar topic for depth information.
  - `/mavros/rangefinder/rangefinder` (Range): Subscribes to the second lidar topic for range information.
  - `/mavros/global_position/local` (Odometry): Subscribes to local position information.

## Parameters
- `rate`: ROS publishing rate in Hz (default: 100 Hz).
- `pwm_min`: Minimum PWM value (default: 1545).
- `pwm_max`: Maximum PWM value (default: 1650).
- `hover_pwm`: PWM value for hover (default: 1560).
- `barro_threshold`: Threshold for barometric height (default: 3.00).
- `lidar_threshold`: Threshold for lidar height (default: 2.00).
- `maintain_threshold`: Threshold for maintaining altitude (default: 1.00).
- `lidar_proximity`: Proximity threshold for the second lidar (default: 1.00).

## PID Controller
The script uses a PID controller to adjust the drone's altitude. The PID parameters are set as follows:
- P (Proportional): 85
- I (Integral): 23
- D (Derivative): 40

## Callbacks
- `lidar_callback1`: Callback for the first lidar topic. Adjusts the altitude based on lidar measurements.
- `lidar_callback2`: Callback for the second lidar topic. Adjusts the altitude based on lidar measurements and proximity conditions.
- `local_callback`: Callback for the local position topic.

## Execution
1. Ensure that ROS is installed and initialized.
2. Run the script using the command: `./terrain_follower_node.py`
3. The node will subscribe to lidar and local position topics, perform terrain following, and publish the throttle PWM signal.

**Note:** Adjust the PID parameters and thresholds as needed for specific drone configurations and environmental conditions.
