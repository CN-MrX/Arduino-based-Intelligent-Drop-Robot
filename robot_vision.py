import cv2 as cv
import numpy as np
import os
from matplotlib import pyplot as plt

from sympy import *
import serial
import time
import math


def getPictureFromCamera():
    
    # Define the output path
    output_dir ='E:/Made By ZYX/Learning/University/TH-2/Robot//'
    
    # Open the target camera
    cap = cv.VideoCapture(1,cv.CAP_DSHOW)

    # Capture a picture
    ret, frame = cap.read()

    # Give the picture a name
    output_path = os.path.join(output_dir,  "capturedPicture.jpg" )

    # Write the picture to the PC
    cv.imwrite(output_path, frame)
    return


def getObjectCenterPostionFromPicture(height):

    # Read the captured picture and resize it to 1280*720
    img = cv.imread("E:/Made By ZYX/Learning/University/TH-2/Robot//capturedPicture.jpg",0)
    img = cv.resize(img,dsize=(1280,720),fx=1,fy=1,interpolation=cv.INTER_LINEAR)

    # Copy the picture as a backup
    img2 = img.copy()

    # Read the template from the PC
    template = cv.imread("E:/Made By ZYX/Learning/University/TH-2/Robot//template.jpg",0)
    w, h = template.shape[::-1]
    
    # Put thw method for comparison in a list
    methods = ['cv.TM_CCOEFF']

    # 
    for meth in methods:

        # Read the backup and standardlize the method
        img = img2.copy()
        method = eval(meth)
        
        # Apply template Matching
        res = cv.matchTemplate(img,template,method)

        # Use a rectangle to select the target object
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
        top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv.rectangle(img,top_left, bottom_right, 255, 2)

        # Calculate the center point of the rectangle
        x_middle=(top_left[0]+bottom_right[0])/2
        y_middle=(top_left[1]+bottom_right[1])/2        
        # Show the results to the console
        print("The pixel value of the center point:")
        print("x=",x_middle)
        print("y=",y_middle)

        # Calculate the real size of the image border
        x_whole_real = 64/75 * float(height)
        y_whole_real = 9/16 * x_whole_real
        # Show the results to the console
        print("The real size of the image border:")
        print("x=",x_whole_real)
        print("y=",y_whole_real)

        # Calculate the position od the object based on real size
        x_final = x_middle/1280*x_whole_real
        y_final = y_middle/720*y_whole_real
        # Show the results to the console
        print("The position based on the real size:")
        print(x_final)
        print(y_final)

        # Define an array to output the result
        position=[0,0]
        position[0]=x_final
        position[1]=y_final

        # Show the result of the machine vision
        plt.subplot(121),plt.imshow(res,cmap = 'gray')
        plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(img,cmap = 'gray')
        plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
        plt.show()

    return position

def coordinateTransform(cameraPosition,x,y):

    # Convert cm to mm
    x_init_value = 10 * float(x)                              
    y_init_value = 10 * float(y)

    # Define an array and covert the camera coordinate system to the robot one
    robotPosition=[0,0]
    robotPosition[0]=-10*cameraPosition[0]+x_init_value
    robotPosition[1]=10*cameraPosition[1]+y_init_value

    return robotPosition



#MAIN FUNCTION

# Open the serial
#s = serial.Serial('COM7', 115200, timeout=5)

# The initialization of the 3 parameters
h = input("Input the height h (cm):")
x = input("Input the offset x (cm):")
y = input("Input the offset y (cm):")

# Main loop
while 1:
    
    # Absorb the character to start or stop the program
    flag=input("Press '1' to start\nPress 'q' to end\n")

    # Start the program
    if flag == '1':

        # Get a picture from the camera
        getPictureFromCamera()

        # Execute the machine vision and get the final position
        result=getObjectCenterPostionFromPicture(h)

        print("Postion based on camera：")
        print(result[0])
        print(result[1])

        # Transform the Postion based on camera to the Postion based on robot
        transformedResult=coordinateTransform(result,x,y)

        print("Postion based on robot：")
        print(transformedResult[0])
        print(transformedResult[1])


        # Execute the robot movement
        #exe(transformedResult[0],transformedResult[1])
        
        #exe(0,200)

    # Stop the program
    if flag == 'q':
        break



'''
# 这是一次去除h参数的尝试，可惜由于精度不够，误差过大，被舍弃，可以写在报告里。

    print(top_left)
    print(bottom_right)

    x_middle=(top_left[0]+bottom_right[0])/2
    y_middle=(top_left[1]+bottom_right[1])/2

    print("中心点显示像素值:")
    print(x_middle)
    print(y_middle)


    x_length=bottom_right[0]-top_left[0]
    y_length=bottom_right[1]-top_left[1]

    print("边框长度像素值:")
    print(x_length)
    print(y_length)

    block_length = (x_length+y_length)/2

    print("边框平均长度像素值:")
    print(block_length)



    x_real = 1280*3.1/block_length
    y_real = 9/16 * x_real

   # x_real=64
    #y_real=36
    
    print("整个画面的长宽（cmin：")
    print(x_real)
    print(y_real)

    x_final = x_middle/1280 *x_real
    y_final = y_middle/720 *y_real

    print("中心点坐标（cmin：")
    print(x_final)
    print(y_final)

'''



