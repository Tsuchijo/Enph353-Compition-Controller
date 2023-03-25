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


##Image callback
def image_callback(msg):
    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # Save your OpenCV2 image as a jpeg 
    cv2.imshow('camera_image.jpeg', cv2_img)
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
