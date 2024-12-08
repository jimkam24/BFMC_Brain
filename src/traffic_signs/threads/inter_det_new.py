import numpy as np
import cv2

# def ROI (image):
#     height = image.shape[0]
#     width = image.shape[1]
#     polygons = np.array([[(0, height), (int(width/2), int(height/2)), (width, height)]]) # triangle mask -> matplotlib: height 0 on top and max on the lowest spot
#     mask = np.zeros_like(image)
#     cv2.fillPoly(mask, polygons, 255)
#     masked_image = cv2.bitwise_and(image, mask)
#     return masked_image

def ROI(image):
    # sets a region of interest in the binary image and uses it as a mask
    height, width = image.shape
    polygons = np.array([[(0, height), (int(width/5), int(2*height/5)), (int(4*width/5), int(2*height/5)), (width, height)]]) 
    #matplotlib: height 0 on top and max on the lowest spot
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


'''
Detects horizontal lines e.g intersections
Returns the image of the intersection and the start and end pixels of the detected intersection
'''
def inter_det(image):
    src = image
    dim = (640, 480)
    
    # resize image
    resized = cv2.resize(src, dim)
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

    bw = cv2.Canny(gray,255, 255,apertureSize = 3) #250, 255
    cropped_image = ROI(bw)

    horizontal = np.copy(cropped_image)
    # Specify size on horizontal axis
    cols = horizontal.shape[1]
    horizontal_size = cols//35 #works for values < cols//30
    # Create structure element for extracting horizontal lines through morphology operations
    horizontalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (horizontal_size, 1))
    # Apply morphology operations
    horizontal = cv2.erode(horizontal, horizontalStructure)
    horizontal = cv2.dilate(horizontal, horizontalStructure)
    contours, hierarchy = cv2.findContours(horizontal,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(resized, contours, -1, (0,255,0), 3)
    max_l = 0
    start = [[0, 0]]
    end = [[0, 0]]
    for cnt in contours:
        l = cv2.arcLength(cnt,True)
        if(l>20):
            if(l>max_l):
                max_l = l
                start = cnt[0]
                end = cnt[len(cnt)//2]
    if start[0][0] == 0 and end[0][0] == 0:
        return
    return horizontal, [start[0][1], start[0][0]], end[0]
    # coordinates = np.column_stack(np.where(horizontal == 255))
    
    # # Check if there are any pixels with value 1
    # if coordinates.shape[0] > 0:
    #     # Find the pixel with the lowest y coordinate
    #     start = coordinates[np.argmin(coordinates[:, 1])]

    #     # Find the pixel with the highest y coordinate
    #     end = coordinates[np.argmax(coordinates[:, 1])]

    #     # print(f"start of intersection (pixel): {start[1], start[0]}")
    #     # print(f"end of intersection (pixel): {end[1], end[0]}")
    #     return horizontal, start, end
    # else:
    #     print("No pixels with value 255 found.")
    #     return
        


'''
With the function below we can estimate distance with accuracy ~5cm.
Estimation works for distances d: 50cm < d < 110cm
Distance is measured from (0,0,0) of the global system of reference, which is below the camera on the car

         |D
         |
      ,--|---,  
   ,--'---:---`--,
  ==(o)-----(o)==J
          ____________ Z
        /|
       / |
      /  |
     /   |
    /    |
   X     Y
'''

def calculate_distance(start):    
    s = 0.81   # scaling parameter | decent values: 1.11-1.13 #1.119
    theta = 18.06   # measure theta (deg) #previous 19.8
    
    # also seem to work 0.75 and 20.12
    
    image_point = np.array([start[1], start[0], 1])
    
    camera_matrix = np.array([  [898.48331233, 0, 319.49999881],    # camera intrinsic parameters
                                [0, 46.36474704, 239.50001243],
                                [0, 0, 1]])       

    theta_rad = np.radians(theta)       #theta in radians
    rotation_matrix = np.array([[1, 0, 0],
                                [0, np.cos(theta_rad), -np.sin(theta_rad)],
                                [0, np.sin(theta_rad), np.cos(theta_rad)]])
                
    translation_matrix = np.array([0, -0.20155644371, 0])   # translation of camera as measured from floor below car center of mass

    coordinates = np.dot(np.linalg.inv(rotation_matrix), (np.dot(np.linalg.inv(camera_matrix), s*image_point.T) - translation_matrix.T))

    return coordinates


