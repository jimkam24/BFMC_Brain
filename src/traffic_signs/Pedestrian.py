import cv2
import numpy as np

def pedestrian(img):
    pedestrian = False
    dimensions = img.shape
    h = int(dimensions[0])
    w = int(dimensions[1])
    img = img[int(h/7): int(h/1) , int(w/4):int(w/1.5)]

    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    #------PINK MASK--------
    lower = np.array([157,73,100]) 
    upper = np.array([172,255,255])
    #lower = np.array([133,80,91]) 
    #upper = np.array([179,255,187])

    pink_mask = cv2.inRange(imgHSV,lower,upper)
    pink_imgResult = cv2.bitwise_and(img,img,mask=pink_mask)
    cv2.imwrite("ped.jpg",pink_imgResult)
    #print(imgResult.size)
    #print(pink_imgResult.size,"pink")

    #detect color
    color ="None"
    number_of_black_pix = np.sum(pink_imgResult == 0)
    if number_of_black_pix<pink_imgResult.size-6000:
        color ="pink"
        pedestrian = True

    if(color=="pink"):
        print("Pedestrian")
    
    return pedestrian
