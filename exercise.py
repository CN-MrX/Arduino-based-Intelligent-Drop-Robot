import cv2 as cv
import numpy as np
import os
from matplotlib import pyplot as plt

from sympy import *
import serial
import time
import math

#from czk
def fun(vector): 
    command = "P"+str(vector[0])+","+str(vector[1])+","+str(vector[2])+","+str(vector[3])+","+str(vector[4])+","+str(vector[5])+","+str(vector[6])+"\n"
    s.write(str.encode(command))
    time.sleep(2)
    return

def initPos():
    fun([0,90,180,90,0,90,30])
    return

def opening():
    fun([0,90,180,90,0,0,30])
    return

def BT(w):
    d1 = 75
    a2 = 125
    a3 = 125
    a4 = 95
    d5 = 150
    
    # q is the vector containing the joint angles
    q = [0,0,0,0,0,0,30]

    #examination of reachability
    if w[0] * w[0] + w[1] * w[1] < 255 * 255 or w[0] * w[0] + w[1] * w[1] > 333 * 333:
        print("out of range")
        return 
    #core algorithm
    q[0] = atan2(w[1],w[0])
    q234 = atan2(- w[3] * cos(q[0]) - w[4] * sin(q[0]),- w[5])
    b1 = w[0] * cos(q[0]) + w[1] * sin(q[0]) - a4 * cos(q234) + d5 * sin(q234)
    b2 = d1 - a4 * sin(q234) - d5 * cos(q234) - w[2]
    bb = b1 * b1 + b2 * b2
    if bb > 250 * 250:
        print("out of range")
        return
    q[2] = acos((bb - a2 * a2 - a3 * a3) / (2 * a2 * a3))
    q[1] = atan2((a2 + a3 * cos(q[2])) * b2 - a3 * b1 * sin(q[2]),(a2 + a3 * cos(q[2])) * b1 + a3 * b2 * sin(q[2]))
    q[3] = q234 - q[1] - q[2]
    q[4] = pi * ln(sqrt(w[3] * w[3] + w[4] * w[4] + w[5] * w[5]))

    #turn results into degree
    q[0] = q[0] = q[0] * 180 / pi
    print("q0:",q[0])
    if q[0] < 78.5 :
        q[0] = q[0] - 8
    else:
        if q[0] <90:
            q[0] = q[0] - 7
        else:
            q[0] = q[0] - 4
    print("q0'':",q[0])
    q[1] = q[1] * 180 / pi + 180
    q[2] = q[2] * 180 / pi + 90
    q[3] = q[3] * 180 / pi + 90
    q[4] = q[4] * 180 / pi

    #turn results into decimal fractions
    q[0] = '%.4f' % q[0] 
    q[1] = '%.4f' % q[1]
    q[2] = '%.4f' % q[2]
    q[3] = '%.4f' % q[3]
    q[4] = '%.4f' % q[4]
    
    return q

def grab(q):
    #q[5] = 55
    q[5] = 70
    return q

def up(q):
    q[1] = 90
    return q

def transfer():
    fun([0,90,180,90,0,55,50])
    return

def release():
    fun([0,90,180,90,0,30,50])
    return

def exe(x,y):
    initPos()
    #You can change the code below
    q = BT([x,y,0,0,0,-1])

    if(q!= None):
        opening()
        fun(q)
        grab(q)
        fun(q)
        up(q)
        fun(q)
        transfer()
        release()
        #Don't change the code below
        initPos()
    return

#from zyx
def getPictureFromCamera():
    
    # Define the output path
    #output_dir ='E:/Made By ZYX/Learning/University/TH-2/Robot//'
    output_dir ='D:/robot/picture//'
    
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
    #img = cv.imread("D:/robot/picture//capturedPicture.jpg",0)
    img = cv.resize(img,dsize=(1280,720),fx=1,fy=1,interpolation=cv.INTER_LINEAR)

    # Copy the picture as a backup
    img2 = img.copy()

    # Read the template from the PC
    template = cv.imread("E:/Made By ZYX/Learning/University/TH-2/Robot//template.jpg",0)
    #template = cv.imread("D:/robot/picture//template.jpg",0)
    w, h = template.shape[::-1]
    
    # Put thw method for comparison in a list
    methods = ['cv.TM_CCOEFF']

    # 
    for meth in methods:

        # Read the backup and standardlize the method
        img = img2.copy()
        method = eval(meth)




        for angle in range(0,361,5):



            
            # Apply template Matching
            res = cv.matchTemplate(img,rotated_1,method)
            


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

            print("res=",res)


            print("minNumber:",minNumber)
            time.sleep(1)

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
s = serial.Serial('COM3', 115200, timeout=5)

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
        #getPictureFromCamera()

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
        exe(transformedResult[0],transformedResult[1])
        #exe(0,280)
        #exe(0,200)

    # Stop the program
    if flag == 'q':
        break

fun([90,90,90,90,90,90,30])

s.close()


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



