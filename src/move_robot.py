#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String

#create cv2 bridge 
bridge = CvBridge()

# Display image in a window called "camera_image.jpeg"
cv2.namedWindow('camera_image.jpeg', cv2.WINDOW_NORMAL)

#create sliders to test values
cv2.createTrackbar('Saturation', 'camera_image.jpeg', 0, 100, lambda x: None)
cv2.createTrackbar('Masking', 'camera_image.jpeg', 0, 255, lambda x: None)


##Image callback
def image_callback(msg):

    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Get the current slider values
    sat = cv2.getTrackbarPos('Saturation', 'camera_image.jpeg') / 100
    mask = cv2.getTrackbarPos('Masking', 'camera_image.jpeg')

    # Convert the image to HSV and adjust the saturation
    hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
    hsv[..., 1] = hsv[..., 1] * sat
    img_sat = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    # Apply a mask to the image
    mask_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    _, mask_img = cv2.threshold(mask_img, mask, 255, cv2.THRESH_BINARY)
    img_masked = cv2.bitwise_and(img_sat, img_sat, mask=mask_img)

    # Display image in a window called "camera_image.jpeg"
    cv2.imshow('win2', cv2_img)

    #wait key
    cv2.waitKey(1)

## Move Robot node
# use ros to publish to the topic /cmd_vel:
def move_robot(x):
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = 0.0
    pub.publish(twist)
    rate.sleep()

# start the timer
def start_timer():
    pub_plate.publish(str('Axolotl,1234,0,0'))

# stop the timer 
def stop_timer():
    pub_plate.publish(str('Axolotl,1234,-1,0'))

rospy.init_node('topic_publisher', anonymous=True)
pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, image_callback)
pub_plate = rospy.Publisher('/license_plate', String, queue_size=1)
rate = rospy.Rate(2) # 10hz
#wait 1 second
rospy.sleep(1)

while not rospy.is_shutdown():
    start_timer()
    move_robot(1)
    rospy.sleep(1)
    move_robot(0)
    stop_timer()
    #sleep forever
    rospy.spin()
