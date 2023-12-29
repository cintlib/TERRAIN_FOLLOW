#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range

class LiDARDataProcessor:
    def __init__(self):
        rospy.init_node('lidar_data_processor', anonymous=True)
        
        # Initialize variables for storing LiDAR data
        self.lidar1_data = None
        self.lidar2_data = None
        
        # Define LiDAR topic names
        self.lidar1_topic = '/mavros/distance_sensor/rangefinder_pub'
        self.lidar2_topic = '/mavros/rangefinder/rangefinder'

        # Create subscribers for LiDAR data
        rospy.Subscriber(self.lidar1_topic, Range, self.lidar1_callback)
        rospy.Subscriber(self.lidar2_topic, Range, self.lidar2_callback)

        # Initialize the weighted average variables
        self.weighted_average = None
        self.weight_lidar1 = 0.9  # Weight for the accurate LiDAR (adjust as needed)
        self.weight_lidar2 = 0.1  # Weight for the other LiDAR (adjust as needed)
        
    def lidar1_callback(self, data):
        # Callback function for LiDAR 1 data
        self.lidar1_data = data

    def lidar2_callback(self, data):
        # Callback function for LiDAR 2 data
        self.lidar2_data = data

    def calculate_weighted_average(self):
        if self.lidar1_data is not None and self.lidar2_data is not None:
            # Calculate the weighted average range
            range_lidar1 = self.lidar1_data.range
            range_lidar2 = self.lidar2_data.range
            
            # Use a weighted average formula
            self.weighted_average = (
                (self.weight_lidar1 * range_lidar1) + 
                (self.weight_lidar2 * range_lidar2)
            ) / (self.weight_lidar1 + self.weight_lidar2)

            
            #  rospy.loginfo("Weighted Average Range: {:.2f} meters".format(self.weighted_average))

    def run(self):
        rate = rospy.Rate(10)  # Set your desired processing rate (Hz)
        while not rospy.is_shutdown():
            self.calculate_weighted_average()
            rate.sleep()

if __name__ == '__main__':
    try:
        lidar_processor = LiDARDataProcessor()
        lidar_processor.run()
        lidar_processor.__setattr__
    except rospy.ROSInterruptException:
        pass
