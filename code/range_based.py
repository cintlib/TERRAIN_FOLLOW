#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from mavros_msgs.msg import Altitude

class LidarFusion:
    def __init__(self):
        rospy.init_node('lidar_fusion_node', anonymous=True)

        # Subscribe to the topics from both lidars
        rospy.Subscriber('/mavros/distance_sensor/rangefinder_pub', Range, self.short_range_callback)
        rospy.Subscriber('/mavros/rangefinder/rangefinder', Range, self.long_range_callback)

        # Create a publisher for the fused data
        self.fused_pub = rospy.Publisher('/fused_lidar', Range, queue_size=10)

        # Initialize variables to store lidar data
        self.short_range_data = None
        self.long_range_data = None

    def short_range_callback(self, data):
        # Callback function for the short-range lidar data
        self.short_range_data = data
        self.fuse_and_publish()

    def long_range_callback(self, data):
        # Callback function for the long-range lidar data
        self.long_range_data = data
        self.fuse_and_publish()

    def fuse_and_publish(self):
        # Fuse the lidar data based on your criteria (e.g., use short-range when close, long-range when far)
        if self.short_range_data is not None and self.long_range_data is not None:
            fused_data = self.fuse_lidar_data(self.short_range_data, self.long_range_data)

            # Print the fused data
            # rospy.loginfo(f"Fused Lidar Data: Range = {fused_data.range}, Min Range = {fused_data.min_range}, Max Range = {fused_data.max_range}")
            rospy.loginfo("Fused Lidar Data: Range = {}, Min Range = {}, Max Range = {}".format(fused_data.range, fused_data.min_range, fused_data.max_range))

            # Publish the fused data
            self.fused_pub.publish(fused_data)

    def fuse_lidar_data(self, short_range_data, long_range_data):
        # Your fusion logic goes here
        # For example, you can compare the distances and choose the appropriate one based on your criteria

        # Example: Use short-range data when close (distance < 5 meters), otherwise use long-range data
        fused_data = short_range_data if short_range_data.range < 5.0 else long_range_data

        return fused_data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        lidar_fusion = LidarFusion()
        lidar_fusion.run()
    except rospy.ROSInterruptException:
        pass
