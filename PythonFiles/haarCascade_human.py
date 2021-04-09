import numpy as np
import cv2 as cv
import argparse
#import serial

def detectAndDisplay(frame):
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)
    frame_gray = cv.GaussianBlur(frame_gray, (5,5), 0)
    bodies =  body_cascade.detectMultiScale(frame_gray)  
    print(bodies)
    for (x,y,w,h) in bodies:
        cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,255),2)
    if len(bodies)!=0:
        #ser.write("LED1_ON".encode())
        print('LED1_ON LED2_OFF')

    cv.imshow('Humans',frame)


parser = argparse.ArgumentParser(description='Code for Cascade Classifier tutorial.')
parser.add_argument('--body_cascade', help='Path to full-body cascade.', default='haarcascade_fullbody.xml')
parser.add_argument('--camera', help='Camera divide number.', type=int, default=0)
args = parser.parse_args()
body_cascade_name = args.body_cascade
body_cascade = cv.CascadeClassifier()

if not body_cascade.load(cv.samples.findFile(body_cascade_name)):
    print('--(!)Error loading body cascade')
    exit(0)

#-- 2. Read the video stream
cap = cv.VideoCapture('green.mp4') 
if not cap.isOpened:
    print('--(!)Error opening video capture')
    exit(0)
while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break
    detectAndDisplay(frame)
    if cv.waitKey(10) == 27:
        break