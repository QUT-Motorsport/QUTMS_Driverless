# importing the necessary libraries
import cv2
import numpy as np

from img_processing import *

# Creating a VideoCapture object to read the video
cap = cv2.VideoCapture('datasets/output2.avi')

# Loop until the end of the video
while (cap.isOpened()):
 
    # Capture frame-by-frame
    ret, frame = cap.read()

    cam_main(frame, display=True)

# release the video capture object
cap.release()
# Closes all the windows currently opened.
cv2.destroyAllWindows()