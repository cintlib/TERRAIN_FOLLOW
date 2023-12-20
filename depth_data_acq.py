#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Bool,Float32

# Initialize ROS node
rospy.init_node('depth_analysis')

# Initialize CvBridge
bridge = CvBridge()

# Initialize the depth data
depth_data = None



def depth_image_callback(msg):
    global depth_data

    # Convert the ROS Image message to a NumPy array
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # depth_image_meters = depth_image * scale_factor

    # Store the depth data
    depth_data = depth_image


# Subscribe to the depth image topic
rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_image_callback)
pub=rospy.Publisher('/depth_camera_value',Float32, queue_size=10)


# Create a function to analyze the middle 100x100 pixel region
def analyze_depth_data(event):
    global depth_data

    if depth_data is not None:
        # Get the height and width of the depth image
        height, width = depth_data.shape

        # Define the region of interest (ROI) in the middle of the image
        roi_start_x = (width) // 2 -50
        roi_start_y = (height) // 2 - 50
        roi_end_x = (width) // 2 + 50
        roi_end_y = (height) // 2 + 50

        # Extract the ROI
        roi_depth_data = depth_data[roi_start_y:roi_end_y, roi_start_x:roi_end_x]/1000

        # Calculate the average depth value in the ROI
        average_depth = np.mean(roi_depth_data)

        # Print the average depth value
        print("Average Depth in ROI: {:.2f} meters".format(average_depth))
        
        pub.publish(average_depth)

# Create a timer to analyze the depth data periodically (adjust the rate as needed)
rospy.Timer(rospy.Duration(0.01), analyze_depth_data)

# Keep the node running
rospy.spin()