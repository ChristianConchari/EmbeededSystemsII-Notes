import cv2
import numpy as np
import serial

#ser = serial.Serial('/dev/ttyACM0', 115200)
#ser.timeout=0.3
#print(ser.portstr)

cap = cv2.VideoCapture('/home/pi/Desktop/RedRabbitGreenGorilla.mp4')

while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    # rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rgb = frame
    # define range of blue color in RGB
    lower_green = np.array([0,30,0])
    upper_green = np.array([15,117,15])

    upper_red = np.array([97,124,255])
    lower_red = np.array([0,0,75])

    # Threshold the HSV image to get only blue colors
    mask1 = cv2.inRange(rgb, lower_green, upper_green)
    mask2 = cv2.inRange(rgb, lower_red, upper_red)
    x1,y1 = mask1.shape
    x2,y2 = mask2.shape
    percentage1 =  ((np.sum(mask1)/255)/(x1*y1))*100
    percentage2 =  ((np.sum(mask2)/255)/(x2*y2))*100
    print("Percentage Green", percentage1)
    print("Percentahe Red", percentage2)
    if percentage1 > 8:
        #ser.write("LED1_ON".encode())
        print('LED1_ON')
    else:
        #ser.write("LED1_OFF".encode())
        print('LED1_OFF')
    if percentage2 > 8:
        #ser.while('LED2_ON'.encode())
        print('LED2_ON')
    else:
        #ser.write("LED2_OFF".encode())
        print('LED2_OFF')

    # Bitwise-AND mask and original image
    res1 = cv2.bitwise_and(rgb,rgb, mask= mask1)
    res2 = cv2.bitwise_and(rgb,rgb, mask= mask2)
    cv2.imshow('Original',rgb)
    cv2.imshow('Green mask',mask1)
    cv2.imshow('Red mask',mask2)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

ser.close()
cv2.destroyAllWindows()