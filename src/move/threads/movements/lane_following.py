import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
#from src.move.threads.movements.stanley import *
import logging
import base64
from src.move.threads.movements.basic import steer

# uncomment for results 

timer_flag = False
visualization = False

if timer_flag:
    time1 = time.time()

def canny(image):
    #used to extract the edges from the image
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 220, 255) #usually 1:2 or 1:3 on low threshod : high threshold -> we need more strict threshold
    return canny


def ROI(image):
    # sets a region of interest in the binary image and uses it as a mask
    height, width = image.shape
    x1 = 0
    x2 = width
    polygons = np.array([[(x1, height), (int(x2/5), int(2*height/5)), (int(4*x2/5), int(2*height/5)), (x2, height)]]) 
    #matplotlib: height 0 on top and max on the lowest spot
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def perspective_transform(cropped):
    height, width = cropped.shape
    x1 = 0
    x2 = width
    src=np.float32([[x1,int(9*height/10)],[x2,int(9*height/10)],[int(x2/5),int(2*height/5)],[int(4*x2/5),int(2*height/5)]])
    dst=np.float32([[x1,height],[x2,height],[x1,0],[x2,0]])
    M = cv2.getPerspectiveTransform(src, dst)  # M is our transformation matrix
    img_size=(width,height)
    warped = cv2.warpPerspective(cropped, M, img_size, flags=cv2.INTER_LINEAR)  # The transformed image
    Minv = cv2.getPerspectiveTransform(dst, src)  # Minv is the inverse of M
    return warped, Minv


def lane_det(bird_view, original_image, Minv):
    
    if visualization:
        cv2.imshow("BEV", bird_view)
    
    bird_height, bird_width = bird_view.shape
    right_lane_warning = 0
    left_lane_warning = 0
    undefined_lane = 0
    
    """----- Creating Histogram - Finding where the lanes begin -----
    
    in final form pipeline will only need bird_view as argument
    
    1.  After creating a warped binary Image, take a histogram of the bottom half of the image
    2.  Find the peak of the left and right halves of the histogram. These will be the starting point for the left and right lines.
        Check if there is a posibility the lanes exist (non zero max of histogram in each side) 
    3.  Choose a safe distance between the two lanes found. If the lanes are closer, one of the two can't be trusted, find which
        is this and don't use it. If we can't find which lane it is, name it as the right lane for now and set the undefined_lane flag
        to true to test if it is a right lane later
    
    """

    histogram = np.sum(bird_view[int((6*bird_height)/10):,:], axis=0) #we want to create the histogram close to the vehichle, so the error is small
    out_img = np.dstack((bird_view, bird_view, bird_view))*255     # Create an output image to draw on and visualize the result


    midpoint = int(bird_width/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint #adding midpoint, histogram[midpoint:] does not use the first half of the histogram
    if (max(histogram[midpoint:])==0):
        right_lane_warning = 1

    if (max(histogram[:midpoint])==0):
        left_lane_warning = 1
    
    
    safe_pixs = int(bird_width/4) #safe number of pixels distance between the two lanes
    # print(leftx_base, rightx_base)
    if (abs(rightx_base - leftx_base) <= safe_pixs):
        if (rightx_base > midpoint and leftx_base > midpoint):
            left_lane_warning = 1
            rightx_base = max(rightx_base, leftx_base)
            
        elif (leftx_base < midpoint and rightx_base < midpoint):
            right_lane_warning = 1 
            leftx_base = min(rightx_base, leftx_base)
        else:
            right_lane_warning = 0
            left_lane_warning = 1
            undefined_lane = 1
    
        
    """ ----- Sliding Windows - Finding Lanes -----
    1.  Choose the number of sliding windows, set their height and width (width is determined by a margin from the center,
        this means each windows' sides are [center-margin, center+margin]. Finally, set as minpix the minimum number of 
        pixels needed to recenter the next window??
    2.  Identify the x and y positions of all nonzero pixels in the image
    3.  Iterate through each window (the method to choose the indices for the points of the lanes is explained in the 
        loop for the left lane - for the right lane is similar)
    4.  Check if any of the two lanes cannot be trusted, based on how many windows did not have enough pixels in them
        and how many of the "good" windows we can afford to lose
    
    WARNING: LARGE nwindows can result in margin dropping to 0 very quickly, so choose nwindows and margin drop rate wisely
    """
    

    nwindows = 30
    window_height = int(bird_height/nwindows)
    margin = int(bird_width/8) # margin is set so that points of the two lanes cannot be on two windows
    start_margin = margin
    minpix = 5


    nonzero = bird_view.nonzero()
    nonzeroy = np.array(nonzero[0]) # array of y elements
    nonzerox = np.array(nonzero[1]) # array of x elements


    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    
    left_warning = 0
    right_warning = 0
    
    for window in range(nwindows):
        if (left_lane_warning == 0):
        # Identify window boundaries in x and y
            win_y_low = bird_height - (window+1)*window_height
            win_y_high = bird_height - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
                
                if visualization:
                    out_img = cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0, 0, 255), 1)
            
            else:
                left_warning += 1


        if (right_lane_warning == 0):
            win_y_low = bird_height - (window+1)*window_height
            win_y_high = bird_height - window*window_height
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            right_lane_inds.append(good_right_inds)
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
                if visualization:
                    out_img = cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0, 0, 255), 1)
            else:
                right_warning += 1

        margin = int(margin-(0.8*start_margin/nwindows)) #reduce margin , so after 30 iterations margin = 20% of the starting margin
        #margin need to be redused so when the window is far up in the image it wont detect both lanes in the same window


    safe_windows_num = 15 #how many windows can we afford to lose
    if (right_warning>=safe_windows_num):
        right_lane_warning=1
    if (left_warning>=safe_windows_num):
        left_lane_warning=1
        
    if visualization:
        cv2.imshow('windows', out_img)


    """ ----- Finding the lanes ----- 
    1.  For each lane concatenate the arrays of indices, extract the left and right line pixel positions and then fit
        a second order polynomial to the line 
    2.  Keep the points of the left lane in the pts_left and pts_right arrays respectively
    3. If the undefined_lane flag is set to true test if it should be left or right...  
    """
    
    ploty = np.linspace(0, bird_height-1, bird_height)
    if (left_lane_warning == 0):
        left_lane_inds = np.concatenate(left_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        
        left_fit = np.polyfit(lefty, leftx, 2)
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        pts_left = np.int_(np.array(np.transpose(np.vstack([left_fitx, ploty]))))
    
    if (right_lane_warning == 0):
        right_lane_inds = np.concatenate(right_lane_inds)
        
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        right_fit = np.polyfit(righty, rightx, 2)
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        pts_right = np.int_(np.array(np.transpose(np.vstack([right_fitx, ploty]))))

    
    if (undefined_lane == 1 and right_lane_warning == 0):
        if ((right_fit[0]*(bird_width//2) + right_fit[1]) > 0):
            pass
        else:
            right_lane_warning = 1
            left_lane_warning = 0
            pts_left = pts_right
            

    """ ----- Results and Visualization -----
    1.  Find and visualize the current trajectory of the car (if it continued to move in a straight line)
    2.  Visualise the left and right lanes (if they can be trusted)
    3.  Find the desired trajectory of the car:
            a. If both lanes can be trusted the desired trajectory is their average
            b. If any of the two lanes can't be trusted the desired trajectory is the lane that
               can be trusted moved in a certain distance from it
    4.  Find the angle of the error as the derivative of the desired trajectory close to the car
    5.  Visualize the picture of the lanes and the original image with the lanes, the desired and the 
        current trajectory on it.
    6.  Return as a result the distance error in pixels of the desired trajectory from the current trajectory and the 
        angle error in degrees
    
    WARNING: The error in pixels is positive when the desired trajectory is left of the current trajectory.
            The error in degrees is positive when the car needs to turn left 
    
    To see the results visualized set the visualization flag to True
    """
    
    desired_trajectory = []
    desired_trajectory_x = []
    desired_trajectory_y = []
    current_trajectory = []
    degree_error = 0
    
    for y in range(0, int(bird_height/3)):
        current_trajectory.append([int(bird_width/2), bird_height-y])
        

    if visualization:
        lanes = np.zeros(bird_view.shape)
        warp_zero = np.zeros_like(bird_view).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        
        for point in current_trajectory:
            lanes = cv2.circle(lanes,tuple(point),0,255)
            color_warp = cv2.circle(color_warp, tuple(point), 0, (0, 165, 255), 5)
        
        if (left_lane_warning == 0):
            for point in pts_left:
                lanes = cv2.circle(lanes,tuple(point),0,255)
                color_warp = cv2.circle(color_warp, tuple(point), 0, (0, 0, 255), 20)
        if (right_lane_warning == 0):
            for point in pts_right:
                lanes = cv2.circle(lanes,tuple(point),0,255)
                color_warp = cv2.circle(color_warp, tuple(point), 0, (255, 0, 0), 20)


    point_der = (9*bird_height//10)
    
    if (left_lane_warning == 0 and right_lane_warning == 0):
        desired_trajectory = (pts_right+pts_left)//2
        for point in desired_trajectory:
            desired_trajectory_x.append(point[0])
            desired_trajectory_y.append(point[1])
            if visualization:
                lanes = cv2.circle(lanes,tuple(point),0,255)
                color_warp = cv2.circle(color_warp, tuple(point), 0, (128, 0, 128), 5)

        desired_fit = np.polyfit(desired_trajectory_y, desired_trajectory_x, 2)
        degree_error = 2* point_der * desired_fit[0] + desired_fit[1]

    elif (left_lane_warning == 0):
        for point in pts_left:
            point = point + [bird_width//6, 0]
            desired_trajectory.append(point)
            desired_trajectory_x.append(point[0])
            desired_trajectory_y.append(point[1])
            if visualization:
                lanes = cv2.circle(lanes,tuple(point),0,255)
                color_warp = cv2.circle(color_warp, tuple(point), 0, (128, 0, 128), 5)

        desired_fit = np.polyfit(desired_trajectory_y, desired_trajectory_x, 2)
        degree_error = 2* point_der * desired_fit[0] + desired_fit[1]            
        
    elif (right_lane_warning == 0):
        for point in pts_right:
            point = point - [bird_width//6, 0]
            desired_trajectory.append(point)
            desired_trajectory_x.append(point[0])
            desired_trajectory_y.append(point[1])
            if visualization:
                lanes = cv2.circle(lanes,tuple(point),0,255)
                color_warp = cv2.circle(color_warp, tuple(point), 0, (128, 0, 128), 5)

        desired_fit = np.polyfit(desired_trajectory_y, desired_trajectory_x, 2)
        degree_error = 2* point_der * desired_fit[0] + desired_fit[1]            
    else:
        print("no lanes found")
        return
    
    
    if visualization:
        cv2.imshow('lanes', lanes)
        #Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        #Combine the result with the original image
        result = cv2.addWeighted(original_image, 0.8, newwarp, 1, 0)
        cv2.imshow('result', result)
    
    error_in_pixs = current_trajectory[0][0] - desired_trajectory[bird_height-1][0] - 20
    degree_error = np.arctan(degree_error) * (180 / np.pi)
    degree_error = round(degree_error, 1)
    
    # future_degree_error = 2* point_der//3 * desired_fit[0] + desired_fit[1]
    # future_degree_error = np.arctan(future_degree_error) * (180 / np.pi)
    # future_degree_error = round(future_degree_error, 1)
    
    # offset = bird_width//25
    
    # if ((future_degree_error - degree_error) > 20):
    #     print("left turn")
    #     error_in_pixs = error_in_pixs - offset
    # elif ((future_degree_error - degree_error) < -20):
    #     print("right turn")
    #     error_in_pixs = error_in_pixs + offset
        

    return error_in_pixs, degree_error, desired_fit[0], desired_fit[1], desired_fit[2]


def stanley_correction( crosstrack_error,heading_error,K, velocity=16): #crosstrack in pixels
    max_steer = 25
    desired_steering_angle = -heading_error - 57*np.arctan(K*crosstrack_error/float(velocity))
    limited_steering_angle = np.clip(desired_steering_angle, -max_steer, max_steer)
    limited_steering_angle = round(limited_steering_angle,1)
    return limited_steering_angle

def followLane(img, K, speed=16):
    try:
        # print("lane following")
        canny_img = canny(img)
        cropped_image = ROI(canny_img)
        bird, Minv = perspective_transform(cropped_image)
        data = lane_det(bird, img, Minv)
        if data is not None:
            error_in_pixs, error_in_degrees, a2, a1, a0 = data
            # with open('numbers.txt', 'a') as file:
            #     numbers = (a2, a1, a0)
            #     file.write(' '.join(map(str, numbers)) + '\n')
            # print(f"Error in pixels is: {error_in_pixs}")
            # print(f"Error in degrees is {error_in_degrees}")
            if speed != 0:
                # if abs(error_in_degrees) > 10:
                #     K = 0.205
                angle = stanley_correction(error_in_pixs, error_in_degrees, K, speed)
                lane_error = -1*(error_in_pixs//30)
                lane_error = np.clip(lane_error, -3, 3)
                # logging.info("K = %.2f",K)
                # logging.info("pixel error: %.2f degree error: %.2f stanley angle: %.2f",error_in_pixs,error_in_degrees, angle)
                return angle, lane_error
            else: 
                return 0,0
    except:
        print("no lanefollowing")