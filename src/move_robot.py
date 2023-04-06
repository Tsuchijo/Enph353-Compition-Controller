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

#create state machine
state = 0
vert_angle = 0
horizontal_line = False
avg_line_x = 0

# create a state machine which follows the vert angle as long as it does not see a horizontal line, then it will stop at the line
def state_machine():
    global state
    global vert_angle
    global horizontal_line
    global avg_line_x
    #start at state 0 and start timer then move onto state 
    if state == 0:
        start_timer()
        state = 1
    #move forward until it sees a horizontal line
    if state == 1:
        if horizontal_line == False:
            move_robot(0.05, vert_angle/100)
        else:
            state = 2
    #stop at the horizontal line and turn clockwise
    if state == 2:
        if horizontal_line == True:
            if avg_line_x < 1000:
                move_robot(0.01, 0.2)
                print(avg_line_x)
            else:
                state = 3
        else:
         state = 1
    # if it sees a car
    if state == 3:
        move_robot(0.1, 0)
        rospy.sleep(0.1)
        if horizontal_line == False:
            state = 1



##Image callback
def image_callback(msg):
    global vert_angle
    global horizontal_line
    global avg_line_x
    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    #do a threshold based on the grayscale image
    ret, mask = cv2.threshold(cv2_img, 220, 255, cv2.THRESH_BINARY)
    #convert to grayscale
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

    image_cutoff = 600
    horizontal_threshold = 0.25
    #cut off the top half of mask
    new_mask = mask[image_cutoff:, ::]
    new_cv2_img = cv2_img[image_cutoff:, ::]

    #use canny edge detection on the masked image
    new_mask = cv2.Canny(new_mask, 100, 200)

    # detect horizontal lines
    lines = cv2.HoughLinesP(new_mask, 1, np.pi/180, 50, minLineLength=75, maxLineGap=15)
    vertical_lines = lines

    # filter only lines with slope between -0.1 and 0.1
    if lines is not None:
        lines = [line for line in lines if abs((line[0][3] - line[0][1]) / (line[0][2] - line[0][0])) > -horizontal_threshold]
        lines = [line for line in lines if abs((line[0][3] - line[0][1]) / (line[0][2] - line[0][0])) < horizontal_threshold]
    
    # average all the horizontal lines
    if lines is not None:
        if len(lines) > 0:
            avg_line_x = 0
            for line in lines:
                avg_line_x += line[0][0]
                avg_line_x += line[0][2]
            avg_line_x = avg_line_x / (len(lines) * 2)

    # check if there enough horizontal lines to be considered a horizontal line
    if lines is not None:
        if len(lines) > 1:
            horizontal_line = True
        else:
            horizontal_line = False

    # filter only lines from vertical lines with slope greater than 5 or less than -5 
    if vertical_lines is not None:
        vertical_lines = [line for line in vertical_lines if abs((line[0][3] - line[0][1]) / (line[0][2] - line[0][0])) > horizontal_threshold]
    
    # average all the vertical lines
    if vertical_lines is not None:
        # take each line and subtract x1 from x2 and y1 from y2
        vertical_lines_x2 = [(line[0][2] - line[0][0]) for line in vertical_lines]
        vertical_lines_y2 = [(line[0][3] - line[0][1]) for line in vertical_lines]
        # take the average of the x2 and y2
        average_x2 = np.average(vertical_lines_x2)
        average_y2 = np.average(vertical_lines_y2)
        # construct a line with the average x2 and y2
        average_line = np.array([[0, 0, average_x2, average_y2]])
    else:
        average_line = None

    # draw lines on image
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(cv2_img, (x1, y1 + image_cutoff), (x2, y2 + image_cutoff), (255, 0, 0), 2)

    # draw vertical lines on image
    if vertical_lines is not None:
        for line in vertical_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(cv2_img, (x1, y1 + image_cutoff), (x2, y2 + image_cutoff) , (0, 0, 255), 2)
    
    # draw average line on image
    if average_line is not None:
        #check if average is nan
        if np.isnan(average_line).any():
            average_line = np.array([[0, 0, 0, 0]])
        x1, y1, x2, y2 = average_line[0]
        #cast to int
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        cv2.line(cv2_img, (640, 300), (640 + y2, x2 + 300), (255, 255, 0), 2)
        vert_angle = y2

    # Display image in a window called "win3"d
    cv2.imshow('win3', new_mask)
    cv2.imshow('win2', cv2_img)

    cv2.waitKey(1)

## Move Robot node
# use ros to publish to the topic /cmd_vel:
def move_robot(x, z):
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = z
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
rate = rospy.Rate(10) # 10hz
#wait 1 second
rospy.sleep(1)

while not rospy.is_shutdown():
    state_machine()
