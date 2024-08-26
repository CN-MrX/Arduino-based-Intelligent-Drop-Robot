from sympy import *

import serial
import time
import math


'''
    Function for self checking
'''
def selfCheck():
    
    time.sleep(1)
    s.write(b'P90,90,90,90,90,90,100\n')
    
    time.sleep(3)
    s.write(b'P180,90,90,90,90,90,100\n')
    
    time.sleep(2)
    s.write(b'P0,90,90,90,90,90,30\n')
    
    time.sleep(3)
    return

'''
    Function for moving to initial position
'''
def initPosition():
    #Move to initial position
    robWrite([0,0,0,0,0,0,50])
    return


def robWrite(parameters):
    #Construct command string
    '''Note: parameters (index 0~4) are angle of joints;
             parameters[5] is angle of gripper;
             parameters[6] is move velocity
    '''
    command ="P"+str(parameters[0])+","+str(parameters[1]+90)+","+str(180-parameters[2])+","+str(parameters[3]+90)+","+str(parameters[4])+","+str(90-parameters[5])+","+str(parameters[6])+"\n"
    #Send command
    s.write(str.encode(command))
    
    time.sleep(2)
    return

'''
    Function to construct a DH-transform matrx
'''
def TM(di,deltai,ai,alphai):
    #Convert angle to radian
    delta = deltai*math.pi/180
    alpha = alphai*math.pi/180
    #Construct DH-transform matrix
    TM = Matrix([[math.cos(delta),  -math.sin(delta)*math.cos(alpha),   math.sin(delta)*math.sin(alpha),    ai*math.cos(delta)],
                 [math.sin(delta),  math.cos(delta)*math.cos(alpha),    -math.cos(delta)*math.sin(alpha),   ai*math.sin(delta)],
                 [0,                math.sin(alpha),                    math.cos(alpha),                    di],
                 [0,                0,                                  0,                                  1]])
    return TM

'''
    Function to perform forward transformation
    Note: input theta are respected to machanism initial position rather than what we have defined before
'''
def DHT(tcpPosition,theta):
    #Construct 5 DH-transform matries
    '''
        Note: theta = joint angle respect to robot's initial pose defined by us,
              rather than the angle respect to robot's initial pose defined by control unit
    '''
    TM1to0 = TM(75,    180+theta[0],     0,      90)
    TM2to1 = TM(0,      90+theta[1]-90,      125,   180)
    TM3to2 = TM(0,      -90+180-theta[2],     125,   180)
    TM4to3 = TM(0,      180+theta[3]-90,     -95,   90)
    TM5to4 = TM(150,     90+theta[4],      0,      0)
    #Convert tcp position to position respecting to base coordination system
    tcp = TM1to0.multiply(TM2to1).multiply(TM3to2).multiply(TM4to3).multiply(TM5to4).multiply(Matrix([tcpPosition[0],tcpPosition[1],tcpPosition[2],1]))
    #Convert position to integer
    for i in range(len(tcp)):
        tcp[i] = int(tcp[i])
    #Print out position matrix
    pprint(tcp)
    #Convert tcp matrix to position array
    position = [0,0,0]
    for i in range(3):
        position[i] = tcp[i]
    #Print out position array
    #print(position)
    return position

#Function test
DHT([0,0,0],[0,90,180,90,0])
print("\n")
DHT([0,0,0],[90,90,90,90,0])
print("\n")
DHT([0,0,0],[0,0,0,0,0])


#Main function
s = serial.Serial('COM7', 115200, timeout=5)

#selfCheck()
initPosition()

#robWrite([0,60,90,0,0,0,30])
#s.write(b'P0,90,45,90,0,90,30\n')
initPosition()
time.sleep(2)
'''
s.write(b'P90,90,90,90,0,90,30\n')
time.sleep(2)
'''
s.close()

