#!/usr/bin/env python3

import math
import time
import rospy

from std_msgs.msg import Float32, UInt32, Float64MultiArray, String
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import numpy as np

class Terrain_following:

    def __init__(self):
        self.rate = rospy.Rate(100)
        self.terrain_pub = rospy.Publisher('i_drone/terrain_follow', UInt32, queue_size=100)
        self.lidar_sub1 = rospy.Subscriber('/depth_camera_value', Float32, self.lidar_callback1)  # First lidar topic
        self.lidar_sub2 = rospy.Subscriber('/mavros/rangefinder/rangefinder', Range, self.lidar_callback2)  # Second lidar topic
        self.local_pos = rospy.Subscriber('/mavros/global_position/local', Odometry, self.local_callback)

        self.curr_posistion = []
        self.prev_time = time.time()

        self.pwm_min = 1545
        self.pwm_max = 1650
        self.hover_pwm = 1560
        self.err_sum = 0
        self.err_rate = 0
        self.prev_err = 0

        self.barro_threshold = 3.00
        self.lidar_threshold = 2.00
        self.maintain_threshold = 1.00
        self.lidar_proximity = 1.00  #Proximity threshold for second lidar

    def local_callback(self, l_st):
        self.l_pos = np.array([l_st.pose.pose.position.x, l_st.pose.pose.position.y, l_st.pose.pose.position.z])

    def pid(self, pos, r, dt):
        P = 85
        I = 23
        D = 40

        err = r - pos
        if dt != 0:
            self.err_rate = (err - self.prev_err) / dt
        else:
            self.err_rate = 0

        if abs(err) > 2:
            self.err_sum += err
        else:
            self.err_sum = 0

        out = P * err + D * self.err_rate + I * self.err_sum
        pwm = out + self.hover_pwm
        pwm = min(max(pwm, self.pwm_min), self.pwm_max)
        self.prev_err = err
        return pwm

    def pos_update(self, pwm):
        throttle_pwm = UInt32()
        throttle_pwm.data = pwm
        self.terrain_pub.publish(throttle_pwm)

    def lidar_callback1(self, data):
        global baro_height
        lidar_height = data.data
        baro_height = self.l_pos[2]

        det_t = time.time() - self.prev_time

        if lidar_height <= self.lidar_threshold:  # Use the lidar_threshold
            des = baro_height + (self.maintain_threshold - lidar_height)
            if des < self.barro_threshold:
                des = self.barro_threshold

            # print("des_obs (Lidar 1)", des)
            pwm = self.pid(baro_height, des, det_t)
            self.prev_time = time.time()
            self.pos_update(int(pwm))

    def lidar_callback2(self, data):
        global baro_height
        lidar_height = data.range
        baro_height = self.l_pos[2]

        det_t = time.time() - self.prev_time

        if lidar_height <= self.lidar_threshold:  # Use the lidar_threshold
            des = baro_height + (self.maintain_threshold - lidar_height)
            if des < self.barro_threshold:
                des = self.barro_threshold

            print("des_obs (Lidar 2)", des)
            pwm = self.pid(baro_height, des, det_t)
            self.prev_time = time.time()
            self.pos_update(int(pwm))
        elif lidar_height <= self.lidar_proximity:  # Proximity condition for Lidar 2
            # Increase the altitude by 1m if something is within 1m proximity (Lidar 2)
            des = baro_height + 1.0

            pwm = self.pid(baro_height, des, det_t)
            self.prev_time = time.time()
            self.pos_update(int(pwm))
        else:
            print("des (Lidar 2)", baro_height)
            pwm = self.pid(baro_height, self.barro_threshold, det_t)
            self.prev_time = time.time()
            self.pos_update(int(pwm))


if __name__ == '__main__':
    rospy.init_node('terrain_follower_node')
    terrain = Terrain_following()
    rospy.spin()
