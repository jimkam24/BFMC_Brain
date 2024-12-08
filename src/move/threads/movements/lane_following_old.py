import time
import cv2   as cv
import numpy as np
import math
from collections import deque
from src.move.threads.movements.basic import setSpeed, steer, brake
from src.move.threads.movements.PID import *
import logging

maxSpeed = 30.0
maxSteeringAngle = 21.0
maxRadius = 1000
SteerAngleStep = 3.0 # can change later
SpeedStep = 4.0 # can change later
currSpeed = 0.0 # start value of speed, changes with setSpeed

kp1 = 0.081  #working value: 0.081 //FOR SPEED 17-30: 0.081
kd1 = 0.042  #working value: 0.042  //FOR SPEED 17-27: 0.042
ki = 0

kp2 = 0.018
kd2 = 0.0008

detection_thresh = 0.7
offset = 0

#in straights scr points need to form ma table
#in turns change the uppper scr points, bring them closer together so i dont detect the other lane
#on right turns bring upper right src closer, on left turns bring upper left src closer
#move lower src points higher up on the image (ie on 1/3 of image height)
#curvature --> fit a circle in the average of the points of left and right lane in BEV, calculate it's curvature, steer respectively

'''
Returns the radius at a certain point of a polynomial (aka curvature)
INPUT: 1. line  -> polynomial
       2. point -> point of the polynomial (probable the middle index of the points of the polynomial array (pixels))
'''
def ROI(img, horizon_percent):
    #horizon_percent=the part of the picture that we look for lanes
    cutting_height=int((1-horizon_percent)*img.shape[0])
    cropped = img[cutting_height:int(9*img.shape[0]/10), :]
    # cv.imshow('cut', cropped)
    return cropped

def edges(img):
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img_gray = cv.GaussianBlur(img, (5,5), 3, 1)
    # img_gray = cv.bilateralFilter(img, 3, 100, 100)
    canny = cv.Canny(img_gray, 100, 200)
    kernel = np.ones((3,3), np.uint8)
    erosion=cv.erode(canny, kernel, iterations=3)
    dialation=cv.dilate(canny, kernel, iterations=5)
    inv = cv.bitwise_not(erosion)
    morph_image=cv.bitwise_and(inv, dialation)
    return canny

def perspective_transform(canny, ROI):
    height, width = canny.shape
    x1=0
    x2=canny.shape[1]
    src=np.float32([[x1,height],[x2,height],[x1,int((1-ROI)*height)],[x2,int((1-ROI)*height)]])
    #Visualize scr points
    # canny=cv.circle(canny,(x1,height),radius=5,color=170,thickness=-1)
    # canny=cv.circle(canny,(x2,height),radius=5,color=170,thickness=-1)
    # canny=cv.circle(canny,(x1,int((1-ROI)*height)),radius=5,color=170,thickness=-1)
    # canny=cv.circle(canny,(x2,int((1-ROI)*height)),radius=5,color=170,thickness=-1)
    # cv.imshow('points', canny)
    dst=np.float32([[x1,height],[x2,height],[x1,0],[x2,0]])
    M = cv.getPerspectiveTransform(src, dst)
    Minv = cv.getPerspectiveTransform(dst, src)
    img_size=(canny.shape[1],canny.shape[0])
    warped = cv.warpPerspective(canny, M, img_size, flags=cv.INTER_LINEAR)
    return warped,Minv,M

def pipeline(binary_warped,undist_image, Minv): #in final form pipeline will only need binary_warped as arguement
    # After creating a warped binary Image,
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[int((5.6*binary_warped.shape[0])/10):,:], axis=0)
    #6*binary_warped.shape[0])/10 creates a histogram of white pixels only on the 4/10th of the image's height
    #we want to create the histogram close to the vehichle, so the error is small

    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255

    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = int(histogram.shape[0]/2)
    car_midpoint = int(histogram.shape[0]/2)

    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    no_left_lane = ((rightx_base != midpoint) and (leftx_base == 0))
    no_right_lane = ((rightx_base == midpoint) and (leftx_base != 0))

    # if ((no_left_lane and abs(rightx_base-midpoint) > 40) or (no_right_lane and abs(leftx_base-midpoint) > 40)):    #for dotted line
    #     print(1)
    #     histogram = np.sum(binary_warped[int((6*binary_warped.shape[0])/10):,:], axis=0)
    #
    #     leftx_base = np.argmax(histogram[:midpoint])
    #     rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    if (abs(rightx_base-leftx_base) < 50):         #if midpoint is on lane case
        midpoint = int(6*histogram.shape[0]/10)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint


    no_left_lane_updated = ((rightx_base != midpoint) and (leftx_base == 0))
    no_right_lane_updated = ((rightx_base == midpoint) and (leftx_base != 0))

    #if i find no left point near where it was expected, set it to the symetrical to the right point in respect to the midpoint +- 500 pixels
    if (no_left_lane_updated) :  #no left lane found
        leftx_base = midpoint - abs(midpoint-rightx_base) - 500

    if (no_right_lane_updated):   #no right lane found
        rightx_base = midpoint + abs(midpoint-leftx_base) + 500
        #same for right lane

    # Choose the number of sliding windows
    nwindows = 30

    # Set height of windows
    window_height = int(binary_warped.shape[0]/nwindows)

    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    # tuple of arrays of nonzero elements (x, y)
    nonzeroy = np.array(nonzero[0])
    # array of y elements
    nonzerox = np.array(nonzero[1])
    # array of x elements

    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Set the width of the windows +/- margin
    margin = 150
    start_margin = margin

    # Set minimum number of pixels found to recenter window
    minpix = 5

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]

        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))


        # Draw the windows on the visualization image
        out_img = cv.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
        (0, 0, 255), 1)
        out_img = cv.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
        (0, 0, 255), 1)
        #cv.imshow('windows', out_img)
        # visualize windows
        margin -= (0.8*start_margin/nwindows)
        margin = int(margin)
        #reduce margin , so after 30 iterations margin = 20% of the starting margin
        #margin need to be redused so when the window is far up in the image it wont detect both lanes in the same window
        #WARNING: LARGE nwindows (line 72) can result in margin dropping to 0 very quickly, so choose nwindows wisely

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    # print(len(left_lane_inds))

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    if (len(right_lane_inds) == 0):  #no right lane
        righty = nonzeroy[left_lane_inds]
        rightx = nonzerox[left_lane_inds]+3*int(binary_warped.shape[1])

    if (len(left_lane_inds) == 0):    #no left lane
        lefty = nonzeroy[right_lane_inds]
        leftx = nonzerox[right_lane_inds]-3*int(binary_warped.shape[1])

    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )

    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    #ayto allazo gia to mprosta
    real_midpoint = (left_fitx[int(5.9*(binary_warped.shape[0]-1)/10)]+right_fitx[int(7*(binary_warped.shape[0]-1)/10)])//2
    #Visualize real midpoint (middle of lanes = where the car is supposed to be) and car midpoint (where the car actually is)

    error = real_midpoint-car_midpoint #error < 0 --> go left, error > 0 --> go right

    pts_left = np.int_(np.array(np.transpose(np.vstack([left_fitx, ploty]))))
    pts_right = np.int_(np.array(np.transpose(np.vstack([right_fitx, ploty]))))
    #THESE RETURN POINTS OF LEFT AND RIGHT LANE RESPECTIVELY

    
    return error

def followLane(frame, queue, queuesList, flag, speed = 15):
    x1PID = PID([kp1,ki,kd1], offset ,[-22, 22])
    x2PID = PID([kp2,ki,kd2], offset , [-7.5, 7.5])
    try:
        img = frame
        img = ROI(img, 0.7)
        edged = edges(img)
        binary_warped,Minv,M = perspective_transform(edged, 0.7)
        error = pipeline(binary_warped, img, Minv)
        
        if abs(error) >= 50 :
            curr_pidCorrection = x1PID.update(error)
        else:
            curr_pidCorrection = x2PID.update(error)

        queue.popleft()
        queue.append(curr_pidCorrection)
        sum = 0
        for i in range (0, 4):
            sum = sum + queue[i]

        pidCorrection = round(sum/4)
        # pidCorrection = curr_pidCorrection

        if (flag == True):
            # print(pidCorrection)
            steer(queuesList, -pidCorrection)
            # setSpeed(queuesList, speed)
            flag = False
        else:
            pass
            # steer(queuesList, pidCorrection)
            # setSpeed(queuesList, speed)

    except:
        print("except_followLane")
        # setSpeed(queuesList, 8)
        # flag = True
        time.sleep(0.1)
