# SBUS Override Node Documentation

## Overview
This Python script functions as a ROS (Robot Operating System) node to override SBUS (Serial Bus) channels based on specified conditions. It subscribes to the topics for SBUS channel values and terrain following information, processes the data, and publishes overridden SBUS channels to control the drone.

## Dependencies
- ROS (Robot Operating System)
- Python 3

## Node Information
- **Node Name:** sbus_override_node
- **Published Topics:**
  - `/mavros/rc/override` (OverrideRCIn): Publishes overridden SBUS channels to control the drone.
- **Subscribed Topics:**
  - `/i_drone/sbus_channels` (UInt16MultiArray): Subscribes to SBUS channel values.
  - `/i_drone/terrain_follow` (UInt32): Subscribes to terrain following information.

## Parameters
- **Throttle Value:** The script adjusts the throttle channel based on the terrain following information.

## Functionality
1. The script subscribes to SBUS channel values and terrain following information.
2. It checks specific conditions related to SBUS channels (throttle, etc.) and terrain following.
3. If the conditions are met, it overrides the SBUS channels and publishes the new values.
4. The overridden SBUS channels are sent to `/mavros/rc/override` to control the drone.

## Execution
1. Ensure that ROS is installed and initialized.
2. Run the script using the command: `./sbus_override_node.py`
3. The node will subscribe to SBUS channel values and terrain following information, process the data, and override SBUS channels as needed.

## Note
- Adjust the conditions in the script based on your specific requirements.
- Ensure that the necessary dependencies and topics are available before running the script.

**Important:** This script assumes the use of the [mavros package](https://github.com/mavlink/mavros) for communication with the flight controller.
