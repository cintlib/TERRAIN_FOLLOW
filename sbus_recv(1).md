# SBUS Receiver Node Documentation

## Overview
This Python script serves as a ROS (Robot Operating System) node to read SBUS (Serial Bus) data from a UART (Universal Asynchronous Receiver-Transmitter) port, decode the SBUS packet, and publish the decoded channel values. The script subscribes to the SBUS data topic, reads the incoming SBUS packets, and publishes the SBUS channel values on the `/i_drone/sbus_channels` topic.

## Dependencies
- ROS (Robot Operating System)
- Python 3
- NumPy
- pyserial

## Node Information
- **Node Name:** sbus_receiver_node
- **Published Topics:**
  - `/i_drone/sbus_channels` (UInt16MultiArray): Publishes the SBUS channel values.
- **Parameters:**
  - `serial_port`: Specifies the UART port (default: '/dev/ttyTHS1').
  - `baud_rate`: Specifies the baud rate (default: 100000).
- **External Libraries:**
  - `pyserial`: Required for serial communication.

## SBUS Decoding
The script decodes the SBUS packet according to the SBUS protocol, extracting 16 channels, fail-safe status, and lost frame status. The decoded channel values are normalized and published.

## Execution
1. Ensure that ROS is installed and initialized.
2. Install the required dependencies (NumPy, pyserial).
3. Run the script using the command: `./sbus_receiver_node.py`
4. The node will subscribe to SBUS data, read incoming SBUS packets, decode the data, and publish SBUS channel values.

## Note
- Adjust the `serial_port` and `baud_rate` parameters as needed for your setup.
- Ensure that the necessary dependencies are installed before running the script.

**Important:** This script assumes the use of a UART connection for receiving SBUS data. Make sure the serial port is correctly configured.
