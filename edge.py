#!/usr/bin/env python

'''
This sample demonstrates Canny edge detection.

Usage:
  edge.py [<video source>]

  Trackbars control edge thresholds.

'''

# Python 2/3 compatibility
from __future__ import print_function

import cv2
import numpy as np
from time import sleep
from picamera.array import PiRGBArray
from picamera import PiCamera
# relative module
##import video

# built-in module
import sys

resolution = [320,240]
delta_thresh = 5
min_area = 2000
font = cv2.FONT_HERSHEY_SIMPLEX
camera = PiCamera()
camera.resolution = resolution
camera.framerate = 30
rawImage = PiRGBArray(camera, size=resolution) # reference to RAW image capture
avg = None
sleep(0.1) # warm-up time for the camera



if __name__ == '__main__':
##    print(__doc__)
##
##    try:
##        fn = sys.argv[1]
##    except:
##        fn = 0
##
    def nothing(*arg):
        pass

    cv2.namedWindow('yazidedge')
    cv2.createTrackbar('thrs1', 'yazidedge', 2000, 5000, nothing)
    cv2.createTrackbar('thrs2', 'yazidedge', 2000, 5000, nothing)


    for frame in camera.capture_continuous(rawImage, format="bgr", use_video_port=True):


        cap = frame.array  # video that will be displayed on screen!
        cv2.imshow('preview_frame', cap)

        img = cap.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thrs1 = cv2.getTrackbarPos('thrs1', 'yazidedge')
        thrs2 = cv2.getTrackbarPos('thrs2', 'yazidedge')
        edge = cv2.Canny(gray, thrs1, thrs2, apertureSize=5)
        vis = img.copy()
        vis = np.uint8(vis/2.)
        vis[edge != 0] = (0, 255, 0)
        cv2.imshow('yazidedge', vis)



        rawImage.truncate(0)
        key = cv2.waitKey(1) & 0xFF
        if key ==  ord("q"):
            break




##    while True:
##        img = cap.copy()
##        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
##        thrs1 = cv2.getTrackbarPos('thrs1', 'yazidedge')
##        thrs2 = cv2.getTrackbarPos('thrs2', 'yazidedge')
##        edge = cv2.Canny(gray, thrs1, thrs2, apertureSize=5)
##        vis = img.copy()
##        vis = np.uint8(vis/2.)
##        vis[edge != 0] = (0, 255, 0)
##        cv2.imshow('yazidedge', vis)
##
##        ch = cv2.waitKey(5) & 0xFF
##        if ch == 27:
##            break
    cv2.destroyAllWindows()
