import cv2

# open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
# cap = cv2.VideoCapture(0)

# set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

# take frame
ret, frame = cap.read()
if ret == False:
    print(cap)
else:
    # write frame to file
    cv2.imwrite('image11.jpg', frame)
# release camera
cap.release()
