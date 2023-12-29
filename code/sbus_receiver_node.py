#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16MultiArray
import serial
import numpy as np


serial_port = '/dev/ttyTHS1'
baud_rate = 100000


ser = serial.Serial(
        port="/dev/ttyTHS1",
        baudrate=100000,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        timeout=None
    )


NUM_CHANNELS = 18
SBUS_PACKET_SIZE = 25


channels = np.zeros(NUM_CHANNELS, dtype=np.uint16)
failSafeStatus = 0
lostFrameStatus = 0

def decode_sbus_packet(packet):
        global channels, failSafeStatus, lostFrameStatus
        channels = np.zeros(NUM_CHANNELS, dtype=np.uint16)

        channels[0] = ((packet[1] | packet[2] << 8) & 0x07FF)
        channels[1] = ((packet[2] >> 3 | packet[3] << 5) & 0x07FF)
        channels[2] = ((packet[3] >> 6 | packet[4] << 2 | packet[5] << 10) & 0x07FF)
        channels[3] = ((packet[5] >> 1 | packet[6] << 7) & 0x07FF)
        channels[4] = ((packet[6] >> 4 | packet[7] << 4) & 0x07FF)
        channels[5] = ((packet[7] >> 7 | packet[8] << 1 | packet[9] << 9) & 0x07FF)
        channels[6] = ((packet[9] >> 2 | packet[10] << 6) & 0x07FF)
        channels[7] = ((packet[10] >> 5 | packet[11] << 3) & 0x07FF)
        channels[8] = ((packet[12] | packet[13] << 8) & 0x07FF)
        channels[9] = ((packet[13] >> 3 | packet[14] << 5) & 0x07FF)
        channels[10] = ((packet[14] >> 6 | packet[15] << 2 | packet[16] << 10) & 0x07FF)
        channels[11] = ((packet[16] >> 1 | packet[17] << 7) & 0x07FF)
        channels[12] = ((packet[17] >> 4 | packet[18] << 4) & 0x07FF)
        channels[13] = ((packet[18] >> 7 | packet[19] << 1 | packet[20] << 9) & 0x07FF)
        channels[14] = ((packet[20] >> 2 | packet[21] << 6) & 0x07FF)
        channels[15] = ((packet[21] >> 5 | packet[22] << 3) & 0x07FF)


        for i in range(0,16):
             channels[i]=((channels[i]-224)/1.535)+1000

        channels[16] = (packet[23] >> 2) & 0x01
        channels[17]= (packet[23] >> 3) & 0x01


def publish_sbus_data():
    
    global channels, failSafeStatus, lostFrameStatus
    pub = rospy.Publisher('i_drone/sbus_channels', UInt16MultiArray, queue_size=10)
    rospy.init_node('sbus_receiver_node', anonymous=True)
    rate = rospy.Rate(100000)

    while not rospy.is_shutdown():
       
        packet = ser.read(SBUS_PACKET_SIZE)
        
        decode_sbus_packet(packet)

        sbus_msg = UInt16MultiArray()
        sbus_msg.data = channels.tolist()
        pub.publish(sbus_msg)
        #print(sbus_msg)
        rate.sleep()


rospy.on_shutdown(ser.close)

if __name__ == '__main__':
    try:
        publish_sbus_data()
    except rospy.ROSInterruptException:
        pass
