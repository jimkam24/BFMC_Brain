import cv2
import numpy as np

def empty(a):
    pass
def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver


def getContours(img):
    contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    x=0
    y=0
    w=0
    h=0
    objectType = "None"
    for cnt in contours:
        area = cv2.contourArea(cnt)
        #print(area)
        if area>600: #600, #1300
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(cnt,True)
            #print(peri)
            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
            #print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            if objCor ==3: objectType ="Tri"
            elif objCor == 4:
                aspRatio = w/float(h)
                if aspRatio >0.98 and aspRatio <1.03: objectType= "Square"
                else:objectType="Rectangle"
            elif objCor>4 and objCor<9: objectType= "Hexagon"
            elif objCor>=9 : objectType= "Circles"
            else:objectType="None"



            cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,255,0),2) #get this to detect color
            cv2.putText(imgContour,objectType,
                        (x+(w//2)-10,y+(h//2)-10),cv2.FONT_HERSHEY_COMPLEX,0.7,
                        (0,0,0),2)
            return x,y,w,h,objectType





#------------------SHAPE DETECTION-----------------------------------#
path = 'test12.mp4'
cap = cv2.VideoCapture(path)
count=0
while (cap.isOpened()):
    ret, img = cap.read()
    count = count+1
    
    #img = cv2.resize(img, (1060, 640)) #(for mp4) and pov
    try:
        dimensions = img.shape
        
        h = int(dimensions[0])
        w = int(dimensions[1])
        img = img[0:int(h/2.8) , int(w/2.2):w] #int(h/6): int(h/2) , int(w/2.5):w (for avi), 0: int(h/2) , int(w/2):w (for mp4)
        original_img = img
        imgContour = img.copy()

        imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        imgBlur = cv2.GaussianBlur(imgGray,(7,7),1)
        imgCanny = cv2.Canny(imgBlur,60,70)
        #cv2.imshow("canny",imgCanny)
        #cv2.imshow("blur", imgBlur)
        x,y,w,h,shape = getContours(imgCanny)

        imgBlank = np.zeros_like(img)
        #imgStack = stackImages(0.7,([img,imgGray,imgBlur], [imgCanny,imgContour,imgBlank]))

        #cv2.imshow("Stack", imgStack)






        #####################color detection#####################################


        img_cropped = img[y-20:y + h +20, x-20:x + w+20]
        imgHSV = cv2.cvtColor(img_cropped,cv2.COLOR_BGR2HSV)
        color = "None"
        #------RED MASK--------
        lower = np.array([124,90,75]) #[124,90,75] , [0,79,97]
        upper = np.array([179,213,171]) #[179,213,171], [179,205,185]
        red_mask = cv2.inRange(imgHSV,lower,upper)
        red_imgResult = cv2.bitwise_and(img_cropped,img_cropped,mask=red_mask)
        #print(imgResult.size)

        #detect color
        red_color = False
        number_of_black_pix = np.sum(red_imgResult == 0)
        if number_of_black_pix<red_imgResult.size-2000:
            red_color = True

        # ------BLUE MASK--------
        lower = np.array([105, 176, 33]) #[109, 205, 22]
        #lower = np.array([94, 131, 26]) # for pov
        #upper = np.array([133, 252, 135]) # for pov
        #lower = np.array([94, 121, 124]) # for  new pov
        #upper = np.array([135, 255, 221]) # for new pov
        upper = np.array([141, 218, 137]) # [114, 232, 208]
        blue_mask = cv2.inRange(imgHSV, lower, upper)
        blue_imgResult = cv2.bitwise_and(img_cropped, img_cropped, mask=blue_mask)
        # print(imgResult.size)

        # detect color
        blue_color = False
        number_of_black_pix = np.sum(blue_imgResult == 0)
        if number_of_black_pix < blue_imgResult.size-1000:
            blue_color = True

        #------YELLOW MASK--------
        lower = np.array([16,89,100]) #[9,71,140]
        upper = np.array([55,246,181]) #[71,163,250]
        yellow_mask = cv2.inRange(imgHSV,lower,upper)
        yellow_imgResult = cv2.bitwise_and(img_cropped,img_cropped,mask=yellow_mask)
        #print(yellow_imgResult.size)

        #detect color
        yellow_color = False
        number_of_black_pix = np.sum(yellow_imgResult == 0)
        #print("black",number_of_black_pix)
        if number_of_black_pix<yellow_imgResult.size - 800:
            yellow_color = True
        
        ''' 
        #------GREEN MASK--------
        lower = np.array([16,89,100]) #[9,71,140]
        upper = np.array([55,246,181]) #[71,163,250]
        green_mask = cv2.inRange(imgHSV,lower,upper)
        green_imgResult = cv2.bitwise_and(img_cropped,img_cropped,mask=green_mask)
        #print(green_imgResult.size)

        #detect color
        number_of_black_pix = np.sum(green_imgResult == 0)
        #print("black",number_of_black_pix)
        if number_of_black_pix<green_imgResult.size - 2000:
            color ="green"
        '''


        # cv2.imshow("Original",img)
        # cv2.imshow("HSV",imgHSV)
        # cv2.imshow("Mask", mask)
        # cv2.imshow("Result", imgResult)

        if(shape=="Hexagon" and red_color==True):
            print("stop")
            cv2.putText(imgContour, "Stop",
                        (x + (w // 2) +10, y + (h // 2) -70), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 0, 0), 2)
        elif(shape=="Tri" and blue_color==True):
            print("crosswalk")
            cv2.putText(imgContour, "Crosswalk",
                        (x + (w // 2) + 10, y + (h // 2) - 70), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 0, 0), 2)
        elif(shape=="Hexagon" and blue_color==True):
            print("parking")
            cv2.putText(imgContour, "parking",
                        (x + (w // 2) + 10, y + (h // 2) -70), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 0, 0), 2)
        elif((shape=="Rectangle" or shape =="Square" or shape=="Hexagon") and yellow_color==True):
            print("priority")
            cv2.putText(imgContour, "prioriy",
                        (x + (w // 2) + 10, y + (h // 2) -70), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 0, 0), 2)
        elif (shape == "Rectangle" and red_color == True):
            print("no entry")


        imgStack = stackImages(0.8,([img_cropped,imgHSV],[red_mask,red_imgResult],[blue_mask,blue_imgResult],[yellow_mask,yellow_imgResult]))
        #imgStack = stackImages(0.7,([img,imgGray,imgBlur], [imgCanny,imgContour,imgBlank]))
        #cv2.imshow("Stacked Images", imgStack)
        #cv2.waitKey(100)
        #cv2.destroyWindow("Stacked Images") #added this fot pov avi


        show_res = imgContour
        #cv2.imwrite("res"+str(count)+".jpg",imgContour)
    except:
        show_res = img
        #print("No sign detected")
    #cv2.imshow("result",show_res)
    #cv2.waitKey(0)


cap.release()
#cv2.destroyAllWindows()





