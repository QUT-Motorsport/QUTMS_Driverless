# TO run this script, enter a frame path as cmd line arg: python3 color_thresholder_app.py frame01.png
import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('Thresholder_App')

cv2.createTrackbar("VMax", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("VMin", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("SMax", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("SMin", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("HMax", "Thresholder_App",0,179,nothing)
cv2.createTrackbar("HMin", "Thresholder_App",0,179,nothing)

img = cv2.imread('datasets/annotation/annotate_0.png')

scale_percent = 100 # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
  
# resize image
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

cv2.setTrackbarPos("VMax", "Thresholder_App", 255)
cv2.setTrackbarPos("VMin", "Thresholder_App", 0)
cv2.setTrackbarPos("SMax", "Thresholder_App", 255)
cv2.setTrackbarPos("SMin", "Thresholder_App", 0)
cv2.setTrackbarPos("HMax", "Thresholder_App", 179)
cv2.setTrackbarPos("HMin", "Thresholder_App", 0)

while(1):
    vmax=cv2.getTrackbarPos("VMax", "Thresholder_App")
    vmin=cv2.getTrackbarPos("VMin", "Thresholder_App")
    smax=cv2.getTrackbarPos("SMax", "Thresholder_App")
    smin=cv2.getTrackbarPos("SMin", "Thresholder_App")
    hmax=cv2.getTrackbarPos("HMax", "Thresholder_App")
    hmin=cv2.getTrackbarPos("HMin", "Thresholder_App")

    min_ = np.array([hmin,smin,vmin])
    max_ = np.array([hmax,smax,vmax])

    mask = cv2.inRange(hsv, min_, max_)

    thresholded_img = cv2.bitwise_and(img, img, mask= mask)

    cv2.imshow("Thresholder_App",thresholded_img)

    k = cv2.waitKey(1) & 0xFF

    # exit if q or esc are pressed
    if (k == ord('q') or k == 27):
        break

cv2.destroyAllWindows()