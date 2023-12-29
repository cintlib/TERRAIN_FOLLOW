#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16MultiArray,UInt32
from mavros_msgs.msg import OverrideRCIn


def terrain_value(msg):
    global throttle_val
    throttle_val=msg.data

def stabilize_guided(sbus_val):
    sbus_val=list(sbus_val)
    sbus_val[2]=throttle_val
    sbus_val=tuple(sbus_val)
    ch = OverrideRCIn()
    ch.channels=sbus_val
    rc_override.publish(ch)


def callback(data):
    sbus_values=data.data
    
    if(sbus_values[4]<1600 and sbus_values[4]>1400 and sbus_values[7]>1400):
        stabilize_guided(sbus_values)
    else: 
        ch = OverrideRCIn()
        ch.channels=sbus_values
        rc_override.publish(ch)



def override_node():
    rospy.init_node("sbus_override_node",anonymous=True)
    rospy.Subscriber("i_drone/sbus_channels",UInt16MultiArray,callback)
    rospy.Subscriber("i_drone/terrain_follow",UInt32,terrain_value)
    global rc_override
    rc_override = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=100)
    rospy.spin()


if __name__=="__main__":
    override_node()
