#!/usr/bin/env python3

import math
import time
import rospy

from std_msgs.msg import Float32, UInt32, Float64MultiArray
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import numpy as np

class Terrain_following:

    def __init__(self):
        self.rate = rospy.Rate(100)
        self.terrain_pub = rospy.Publisher('i_drone/terrain_follow', UInt32, queue_size=100)
        self.depth_height_sub = rospy.Subscriber('/depth_camera_value', Float32, self.height_callback)
        self.local_pos = rospy.Subscriber('/mavros/global_position/local', Odometry, self.local_callback)
        self.front_lidar_sub = rospy.Subscriber('/mavros/rangefinder/rangefinder', Range, self.front_lidar_callback)

        self.curr_posistion = []
        self.position_error = []
        self.prev_time = time.time()
        self.pos = []

        self.pwm_min = 1545
        self.pwm_max = 1650
        self.hover_pwm = 1560
        self.err_sum = 0
        self.err_rate = 0
        self.prev_err = 0

        self.barro_threshold = 3.00
        self.lidar_threshold = 2.00
        self.maintain_threshold = 1.00
        self.front_lidar_threshold = 1.00

    def local_callback(self, l_st):
        self.l_pos = np.array([l_st.pose.pose.position.x, l_st.pose.pose.position.y, l_st.pose.pose.position.z])
        self.l_vel = np.array([l_st.twist.twist.linear.x, l_st.twist.twist.linear.y, l_st.twist.twist.linear.z])
        lpos = Float64MultiArray()
        lpos.data = [self.l_pos[0], self.l_pos[1], self.l_pos[2]]

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
        p = baro_height
        self.pos.append(p)

    def height_callback(self, data):
        global baro_height

    lidar_height = float(data.data)
    baro_height = self.l_pos[2]
    current_time = time.time()
    det_t = current_time - self.prev_time

    print("Barometric Height: {:.2f} meters".format(baro_height))
    print("camera height : {:.2f} meters".format(lidar_height))

    if data.data != math.inf:
        # Check for obstacles detected by the depth camera
        if lidar_height <= self.lidar_threshold:
            des = baro_height + (self.maintain_threshold - lidar_height)
            if des < self.barro_threshold:
                des = self.barro_threshold
            pwm = self.pid(baro_height, des, det_t)
            self.prev_time = current_time
            self.pos_update(int(pwm))

        # Check for obstacles detected by the front-facing lidar
        front_obstacle_distance = data.range
        if front_obstacle_distance < self.front_lidar_threshold:
            des = baro_height + (self.maintain_threshold - front_obstacle_distance)
            if des < self.barro_threshold:
                des = self.barro_threshold
            pwm = self.pid(baro_height, des, det_t)
            self.prev_time = current_time
            self.pos_update(int(pwm))

    def front_lidar_callback(self, data):
        front_obstacle_distance = data.range
        print("Obstacle Distance from Front Lidar: {:.2f} meters".format(front_obstacle_distance))
        current_time = time.time()
        det_t = current_time - self.prev_time

        if front_obstacle_distance < self.front_lidar_threshold:
            des = baro_height + (self.maintain_threshold - front_obstacle_distance)
            if des < self.barro_threshold:
                des = self.barro_threshold
            pwm = self.pid(baro_height, des, det_t)
            self.prev_time = current_time
            self.pos_update(int(pwm))

if __name__ == '__main__':
    rospy.init_node('terrain_follower_node')
    terrain = Terrain_following()
    rospy.spin()
