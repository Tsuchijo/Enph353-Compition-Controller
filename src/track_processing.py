import numpy as np
import cv2

# HSV values for the track
h_low = 20
h_high = 40
s_low = 37
s_high = 100
v_low = 179
v_high = 211


def treshold(h_low, s_low, v_low, h_high, s_high, v_high, img, minLineLength=70, maxLineGap=50):
    img_copy = img.copy()
    #convert the image to HSV and threshold it
    blurred = cv2.GaussianBlur(img_copy, (31, 31), 0)
    frame_HSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (h_low, s_low, v_low), (h_high, s_high, v_high))
    #do the phough transform on the thresholded image
    edges = frame_threshold
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength, maxLineGap)
    #draw the lines on the image
    angle = 0
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # take the angle of the lines
        slopes = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slopes.append(np.arctan((x2-x1)/(y2-y1)))
        # take the average of the angles
        angle = np.mean(slopes)
        #draw the angle on the image
        cv2.putText(img_copy, str(angle), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    #stack the images
    edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    frame_threshold = np.hstack((img_copy, edges))
    #reduce the size of the image
    frame_threshold = cv2.resize(frame_threshold, (0,0), fx=0.5, fy=0.5)
    cv2.imshow('image', frame_threshold)

def h_low_callback(x):
    global h_low
    h_low = x
    treshold(h_low, s_low, v_low, h_high, s_high, v_high, img)

def h_high_callback(x):
    global h_high
    h_high = x
    treshold(h_low, s_low, v_low, h_high, s_high, v_high, img)

def s_low_callback(x):
    global s_low
    s_low = x
    treshold(h_low, s_low, v_low, h_high, s_high, v_high, img)

def s_high_callback(x):
    global s_high
    s_high = x
    treshold(h_low, s_low, v_low, h_high, s_high, v_high, img)

def v_low_callback(x):
    global v_low
    v_low = x
    treshold(h_low, s_low, v_low, h_high, s_high, v_high, img)

def v_high_callback(x):
    global v_high
    v_high = x
    treshold(h_low, s_low, v_low, h_high, s_high, v_high, img)

def minLineLength_callback(x):
    global minLineLength
    minLineLength = x
    treshold(h_low, s_low, v_low, h_high, s_high, v_high, img, minLineLength, maxLineGap)

def maxLineGap_callback(x):
    global maxLineGap
    maxLineGap = x
    treshold(h_low, s_low, v_low, h_high, s_high, v_high, img, minLineLength, maxLineGap)

def nothing(x):
    pass

# open the images located in the images folder
img = cv2.imread('images/track2.png')

# show the image
cv2.imshow('image', img)

# add sliders to the window
cv2.createTrackbar('Hue Min', 'image', 0, 179, h_low_callback)
cv2.createTrackbar('Hue Max', 'image', 0, 179, h_high_callback)
cv2.createTrackbar('Sat Min', 'image', 0, 255,  s_low_callback)
cv2.createTrackbar('Sat Max', 'image', 0, 255,  s_high_callback)
cv2.createTrackbar('Val Min', 'image', 0, 255,  v_low_callback)
cv2.createTrackbar('Val Max', 'image', 0, 255,  v_high_callback)
cv2.createTrackbar('Min Line Length', 'image', 0, 100, maxLineGap_callback)
cv2.createTrackbar('Max Line Gap', 'image', 0, 100, minLineLength_callback)
cv2.waitKey(0)
cv2.destroyAllWindows()
