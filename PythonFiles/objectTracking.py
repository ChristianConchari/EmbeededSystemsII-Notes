import serial
#init serial port and bound
# bound rate on two ports must be the same
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.timeout=0.3
## cv2
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

flag = False
p_flag = False

while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to RGB
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # define range of blue color in RGB
    lower_green = np.array([69,158,111])
    upper_green = np.array([54,121,85])

    lower_red = np.array([69,158,111])
    upper_red = np.array([54,121,85])


    # Threshold the RGB image to get only blue colors
    mask = cv2.inRange(rgb, upper_blue, lower_blue)
    x,y = mask.shape
    left = mask[0:int(x), 0:int(y/2)-1]

    per = ((np.sum(left)/255)/(x*y))*100
    # print("Percentage", per)

    p_flag = flag

    if per > 0.001:
        flag = True
    else:
        flag = False

    if p_flag < flag:
        ser.write("LD_ON")
        print("LD_ON")
    if p_flag > flag:
        ser.write("LD_OFF")
        print("LD_OFF")

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()