#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState
import os
import keras

##create cv2 bridge
bridge = CvBridge()

def callback(data):
    # convert the image to cv2 format
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # display the image
    cv2.imshow('image', cv_image)
    pred = (model.predict(np.array([cv_image])))
    action = (np.argmax(pred))
    if action == 0:
        move_robot(0, 0.2)
    elif action == 1:
        move_robot(0, -0.2)
    elif action == 2:
        move_robot(0.2, 0)
    elif action == 3:
        move_robot(0.0, 0)
    cv2.waitKey(1)

##subscribe to the camera
def subscribe_to_camera():
    rospy.init_node('get_training_data', anonymous=True)
    rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback)

##init the keras model
def init_model():
    model = keras.models.load_model("/home/fizzer/ros_ws/src/Enph353-Compition-Controller/src/Models/drive_model")
    model.summary()
    return model

## Move Robot node
# use ros to publish to the topic /cmd_vel:
def move_robot(x, z):
    # handle casse where x or z is None or not a number
    if x is None or not isinstance(x, (int, float)):
        x = 0
    if z is None or not isinstance(z, (int, float)):
        z = 0
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = z
    pub.publish(twist)
    rate.sleep()
    rospy.sleep(0.2)

subscribe_to_camera()
pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
model = init_model()
rate = rospy.Rate(10) # 10hz

## loop to get the data
while not rospy.is_shutdown():
    rospy.spin()