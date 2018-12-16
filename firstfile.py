# CannyStill.py

import cv2
import numpy as np
import matplotlib.pyplot as plt

#set up webcam 0 as "cap"
camera = cv2.VideoCapture(0)

#loop to open frame and show what is in webcam in the frame over and over
while True:
    ret, frame = camera.read()
    frame1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('gray video feed', frame1)
    cv2.imshow('video Feed',frame)


# if q button is pressed close the frame
    if cv2.waitKey(1) & 0xFF== ord('q'):
        break
camera.release()
cv2.destroy


