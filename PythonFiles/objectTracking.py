import cv2
import numpy as np
import serial

ser = serial.Serial('/dev/ttyACM0', 115200)
ser.timeout=0.3
print(ser.portstr)

cap = cv2.VideoCapture(0)

while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    lower_blue1 = np.array([110,0,0])
    upper_blue1 = np.array([230,70,70])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(frame, lower_blue1, upper_blue1)
    x,y = mask.shape
    left = mask[0:int(x), 0:int(y/2)-1]
    percentage =  ((np.sum(left)/255)/(x*y))*100
    print("Percentage",percentage)
    if percentage > 8:
        ser.write("LD_ON".encode())
        print('blue')
    else:
        ser.write("LD_OFF".encode())
        print('No blue')
    
    cv2.imshow('left',left)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

ser.close()
cv2.destroyAllWindows()