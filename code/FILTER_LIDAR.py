import rospy
from sensor_msgs.msg import Range

class LidarFusionFilter:
    def __init__(self):
        rospy.init_node('lidar_fusion_node', anonymous=True)
        rospy.Subscriber('/mavros/distance_sensor/rangefinder_pub', Range, self.short_range_callback)
        rospy.Subscriber('/mavros/rangefinder/rangefinder', Range, self.long_range_callback)
        self.fused_pub = rospy.Publisher('/fused_lidar', Range, queue_size=10)
        self.short_range_data = None
        self.long_range_data = None

        # Filter parameters
        self.alpha = 0.98  # Complementary filter coefficient

    def short_range_callback(self, data):
        self.short_range_data = data
        self.fuse_and_publish()

    def long_range_callback(self, data):
        self.long_range_data = data
        self.fuse_and_publish()

    def fuse_and_publish(self):
        if self.short_range_data is not None and self.long_range_data is not None:
            fused_data = self.fuse_lidar_data(self.short_range_data.range, self.long_range_data.range)
            rospy.loginfo("Fused Lidar Data: Range = {}".format(fused_data.range))

            # Publish the fused data
            self.fused_pub.publish(fused_data)

    def fuse_lidar_data(self, short_range, long_range):
        # Complementary filter
        fused_range = self.alpha * short_range + (1 - self.alpha) * long_range

        fused_data = Range()
        fused_data.range = fused_range
        fused_data.min_range = min(short_range, long_range)
        fused_data.max_range = max(short_range, long_range)

        return fused_data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        lidar_fusion = LidarFusionFilter()
        lidar_fusion.run()
    except rospy.ROSInterruptException:
        pass
