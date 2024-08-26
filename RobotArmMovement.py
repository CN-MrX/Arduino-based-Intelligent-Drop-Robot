from sympy import *
import serial
import time
import math

def fun(vector):
    command = "P"+str(vector[0])+","+str(vector[1])+","+str(vector[2])+","+str(vector[3])+","+str(vector[4])+","+str(vector[5])+","+str(vector[6])+"\n"
    s.write(str.encode(command))
    time.sleep(2)
    return

def initPos():
    fun([0,90,180,90,0,90,50])
    return

def BT(w):
    d1 = 75
    a2 = 125
    a3 = 125
    a4 = 95
    d5 = 150
    
    # q is the vector containing the joint angles
    q = [0,0,0,0,0,0,50]

    #core algorithm
    q[0] = atan2(w[1],w[0])
    q234 = atan2(- w[3] * cos(q[0]) - w[4] * sin(q[0]),- w[5])
    b1 = w[0] * cos(q[0]) + w[1] * sin(q[0]) - a4 * cos(q234) + d5 * sin(q234)
    b2 = d1 - a4 * sin(q234) - d5 * cos(q234) - w[2]
    bb = b1 * b1 + b2 * b2
    if bb > 250 * 250:
        print("戳啦")
        return
    q[2] = acos((bb - a2 * a2 - a3 * a3) / (2 * a2 * a3))
    q[1] = atan2((a2 + a3 * cos(q[2])) * b2 - a3 * b1 * sin(q[2]),(a2 + a3 * cos(q[2])) * b1 + a3 * b2 * sin(q[2]))
    q[3] = q234 - q[1] - q[2]
    q[4] = pi * ln(sqrt(w[3] * w[3] + w[4] * w[4] + w[5] * w[5]))

    #turn results into degree
    q[0] = q[0] * 180 / pi
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
    q[5] = 55
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

    fun(q)
    grab(q)
    fun(q)
    transfer()
    release()
    #Don't change the code below
    initPos()
    s.close
    return

#test
s = serial.Serial('COM7', 115200, timeout=5)
exe(-20,242)
