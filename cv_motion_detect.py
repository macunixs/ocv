from time import sleep
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import imutils
import time
import datetime
resolution = [320,240]
start_clk = time.time()
x , y , w , h = 0 , 0 , 0 , 0
centre = 0
count = 0
tick = 0
delta_thresh = 5
min_area = 2000
font = cv2.FONT_HERSHEY_SIMPLEX
camera = PiCamera()
camera.resolution = resolution
camera.framerate = 30
rawImage = PiRGBArray(camera, size=resolution) # reference to RAW image capture
avg = None
sleep(0.1) # warm-up time for the camera

##camera.capture(rawImage, format="bgr")      # grab image from sensor

#from here ¨array¨ pointer is very crucial so that cv2 can display the image using imshow()
#array pointer can be placed either inside or outside the imshow() function

##image = rawImage.array
##print(image) # location of array in memory without pointer / numerical array with pointer
##cv2.imshow("ImageBionzX", image)
##cv2.waitKey(0)

# the function below will capture RAW images continuously to create video
for frame in camera.capture_continuous(rawImage, format="bgr", use_video_port=True):
    count=count+1
    clock = time.time() - start_clk
    tick+=clock
    image = frame.array  # video that will be displayed on screen!

    timestamp = datetime.datetime.now()
    text = 'NO MOVEMENT DETECTED'
    crosshair = '+'

##    ret,trash = cv2.threshold(image,127,255,0)
##    kontor,hierarchy = cv2.findContours(image, 1,2)
##    cnt = kontor[0]
##    M = cv2.moments(cnt)
##    print(M)

    #resize frame  and convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(21,21),0)

    # if the average frame is None, initialize it
    if avg is None:
            print ("[INFO] starting background model...")
            avg = gray.copy().astype("float")
            rawImage.truncate(0)
            continue
        
    # accumulate the weighted average between the current frame and
    # previous frames, then compute the difference between the current
    # frame and running average
    cv2.accumulateWeighted(gray, avg, 0.5)
    frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
    
    # threshold the delta image, dilate the thresholded image to fill
    # in holes, then find contours on thresholded image
    thresh = cv2.threshold(frameDelta, delta_thresh, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    ( _ , cnts , _ ) = cv2.findContours(     thresh.copy(),
                                                               cv2.RETR_EXTERNAL,
                                                               cv2.CHAIN_APPROX_SIMPLE      )

    # loop over the contours
    for c in cnts:
            # if the contour is too small, ignore it
            if cv2.contourArea(c) < min_area :
                    continue

            # compute the bounding box for the contour, draw it on the frame,
            # and update the text
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            centre = [(x+w)/2 , (y+h)/2]

            print('x:{0}   |   y:{1}'.format(x,y))
            print('centre:{}'.format(centre))

##            rect = cv2.minAreaRect(c)
##            box = cv2.boxPoints(rect)
##            box = np.int0(box)
##            print('x:{0}'.format(box))
##
##            cv2.drawContours(image,[box],0,(0,0,255),2)

            text = "INTRUDER DETECTED"

    # draw the text and timestamp on the frame
    cv2.putText(image, "{}".format(crosshair), (int((x+w)/2),int((y+h)/2)), font , 0.5, (100, 255, 0), 1,cv2.LINE_AA)

    ts = timestamp.strftime("%A %d %B %sY %I:%M:%S%p")
    cv2.putText(image, "Room Status: {}".format(text), (10, 20), font , 0.5, (100, 255, 0), 1,cv2.LINE_AA)
    cv2.putText(image, "centre: {}".format(centre), (10, 50), font , 0.5, (100, 255, 0), 1,cv2.LINE_AA)

    cv2.putText(image, ts, (10, image.shape[0] - 10), font, 0.45, (255, 255, 0), 1,cv2.LINE_AA)

        
##    cv2.imshow("RAWframeDelta" , frameDelta)

    cv2.imshow("RAWbgr" , image)
##    cv2.imshow("RAWgray" , gray)
##    print('image:{0}--------time:{1}'.format(count,tick))
    # [IMPORTANT] now we have to clear the stream to prepare for the next frame!
    rawImage.truncate(0)
    key = cv2.waitKey(1) & 0xFF


    start_clk = time.time()
    if key ==  ord("q"):
        break

##with PiCamera() as camera:
##    with PiRGBArray(camera) as output:
##        camera.resolution = (1280, 720)
##        camera.capture(output, 'bgr')
##        print('Captured %dx%d image' % (output.array.shape[1], output.array.shape[0]))
##        image = output.array
##        cv2.imshow('bionzxxx',image)
##        cv2.waitKey(0)
##        output.truncate(0)
##        camera.resolution = (640, 480)
##        camera.capture(output, 'bgr')
##        image = output.array
##        cv2.imshow('bionzxxx',image)
##        cv2.waitKey(0)
##        print('Captured %dx%d image' % (output.array.shape[1], output.array.shape[0]))
