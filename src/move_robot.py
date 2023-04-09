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
lines_pointing_away = False
lines_bimodal = False
car_timer = 0
see_car = False

# create a state machine to control the car movement
# state 0: start timer
# state 1: line follow off vertical lines until it sees a horizontal line or the lines point away from eachother
# state 2: decision making state for if it sees a horizontal line, if the line is in the center then it is a left turn
# if the line is to the right or left it is a car, if the lines are to the left and right but not center it is an intersection
# state 3: if it sees a car then it will move forward for a set amount of time and then go back to state 1
# state 4: if it sees a left turn then it will turn left until it no longer sees a horizontal line then go back to state 1
# state 5: if it sees an intersection it will go forward until it sees a horizontal line then go back to state 2
# state 6: if it sees the lines pointing away from eachother then it will turn left until it sees a horizontal line then go back to state 1

def state_machine():
    global state
    global vert_angle
    global horizontal_line
    global avg_line_x
    global lines_pointing_away
    global lines_bimodal
    global car_timer
    global see_car
    rospy.sleep(0.1)
    #start at state 0 and start timer then move onto state 
    if state == 0:
        start_timer()
        move_robot(0.1, 0)
        state = 6
    #move forward until it sees a horizontal line
    if state == 1:
        if horizontal_line == False:
            move_robot(0.05, vert_angle/100)
        else:
            state = 2
    #decision making state
    if state == 2:
        print('line x', avg_line_x)
        #if the line is in the center then it is a left turn
        if avg_line_x > 300 and avg_line_x < 1000:
            state = 4
        #if the line is to the right or left it is a car or intersection
        elif avg_line_x < 300 or avg_line_x > 1000:
            if see_car == True:
                car_timer += 1
                if car_timer > 5:
                    print('car')
                    state = 3
                    car_timer = 0
            else:
                state = 5
        else:
            state = 1
    # if it sees a car
    if state == 3:
        move_robot(0.1, 0)
        rospy.sleep(3)
        state = 1
    # if it sees a left turn
    if state == 4:
        move_robot(0.01, 0.5)
        if horizontal_line == False:
            state = 1
    # if it sees an intersection
    if state == 5:
        move_robot(0.1, 0)
        rospy.sleep(0.1)
        state = 6
    # if it sees the lines pointing away from eachother
    if state == 6:
        if lines_pointing_away == True:
            move_robot(0.1, 0)
        else:
            state = 1



##Image callback
def image_callback(msg):
    global vert_angle
    global horizontal_line
    global avg_line_x
    global lines_pointing_away
    global lines_bimodal
    global see_car
    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    #do a threshold based on the grayscale image
    ret, mask = cv2.threshold(cv2_img, 220, 255, cv2.THRESH_BINARY)
    #convert to grayscale
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    
    image_cutoff = 500
    horizontal_threshold = 0.25
    #cut off the top half of mask
    new_mask = mask[image_cutoff:, ::]
    new_cv2_img = cv2_img[image_cutoff - 50:, ::]
    # for each pixel in the image if the blue is 100 more than red and green then it is a car
    car_mask = np.where((new_cv2_img[:,:,0] - new_cv2_img[:,:,1] > 100) & (new_cv2_img[:,:,0] - new_cv2_img[:,:,2] > 100), 255, 0)
    #convert from bool to int
    car_mask = car_mask.astype(float)
    #sum all the pixels in the mask
    car_mask_sum = np.sum(car_mask)
    #if the sum is greater than 10 then it is a car
    if (car_mask_sum / 255) > 500:
        see_car = True
    else:
        see_car = False
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

    # check if the vertical lines are pointing away from the center of the image
    if vertical_lines is not None:
        # split the lines into ones on left and right half of image
        left_vertical_lines = [line for line in vertical_lines if line[0][0] < 640/2]
        right_vertical_lines = [line for line in vertical_lines if line[0][0] > 640/2]

        # check if there are enough lines on each side
        if len(left_vertical_lines) > 1 and len(right_vertical_lines) > 1:
            # check if the lines are pointing away from the center of the image
            for line in left_vertical_lines:
                if line[0][1] < line[0][3]:
                    lines_pointing_away = True
                else:
                    lines_pointing_away = False
                    break
            for line in right_vertical_lines:
                if line[0][1] > line[0][3]:
                    lines_pointing_away = True
                else:
                    lines_pointing_away = False
                    break
        else:
            lines_pointing_away = False

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
            vert_angle = 0
        x1, y1, x2, y2 = average_line[0]
        #cast to int
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        cv2.line(cv2_img, (640, 300), (640 + y2, x2 + 300), (255, 255, 0), 2)
        vert_angle = (y2 + vert_angle)/2
    else:
        vert_angle = 0

    # Display image in a window called "win3"d
    cv2.imshow('win3', car_mask)
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
    print('state: ' + str(state))
    state_machine()
