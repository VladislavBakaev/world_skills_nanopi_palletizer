#!/usr/bin/env python

from math import *
import numpy as np

class RoboticArm:
    def __init__(self):
        self.__l1 = 148.78
        self.__l2 = 21.116
        self.__l3 = 148.00
        self.__l4 = 160.002
        self.__l5 = 90.0
        self.__l6 = 135.0
        self.__l7 = 6.0
        
        self.__convert_const = 2048/pi
        self.__convert_const_AX = 512/pi
        self.__const1 = 2048
        self.__const2 = 1930
        self.__const3 = 2230
        self.__const4 = 512

    def DirectProblem(self,q1,q2,q3,q4,q5):
        l1 = self.__l1
        l2 = self.__l2
        l3 = self.__l3
        l4 = self.__l4
        l5 = self.__l5
        l6 = self.__l6
        l7 = self.__l7

        d = sqrt(l3*l3+l4*l4-2*l3*l4*cos(s1+s2))
        gamma = acos((l3*l3+d*d-l4*l4)/(2*l3*d))
        beta = pi/2 - s1 - gamma
        print "gamma: "+str(gamma*180/pi)
        print "beta: "+str(beta*180/pi)
        x = d*cos(beta) + l2 + l5
        z = d*sin(beta) + l1 - l6
        y = x*sin(s0) + sin(q5)*l7
        x = x*cos(s0) + cos(q5)*l7
        

        return (np.array([x,y,z,1]))


    def __VadatingOfJointAngle(self,alpha2,gamma3,alpha3):
        if alpha2 < -pi/4:
            #no point
            return False
        elif alpha2 >= -pi/4 and alpha2 <= 0 and gamma3 <= pi/1.9 and gamma3 >= pi/10:
            #point exist
            return True
        elif alpha2 <= pi/1.05 and gamma3 <= pi/1.05 and gamma3>=pi/10:
            #point exist
            return True
        else:
            return False
            #no point

        
    

    def InversProblem(self,X,Y,Z,pitch = 0):
        l1 = self.__l1
        l2 = self.__l2
        l3 = self.__l3
        l4 = self.__l4
        l5 = self.__l5
        l6 = self.__l6
        l7 = self.__l7
        try:
            z = Z+l6-l1
            # print "z: "+str(z)
            if X != 0:
              alpha1 = atan2(Y,X)
            else:
              alpha1 = pi/2

            x = X/cos(alpha1)
            x = x - l2 - l5 
            # print "x: "+str(x)
            d = sqrt(x*x+z*z)
            gamma = acos((l3*l3+d*d-l4*l4)/(2*l3*d))
            beta = gamma + atan(z/x)
            alpha2 = pi/2 - beta

            gamma1 = acos((l3*l3+l4*l4-d*d)/(2*l3*l4))
            
            alpha3 = gamma1 - alpha2
            # if self.__VadatingOfJointAngle(alpha2 ,gamma1, alpha3) == False:
            #     return False, (0,0,0,0)
            # print("gamma1: ", gamma1*180/pi)
            # print("alpha1: ", alpha1)
            # print("alpha2: ", alpha2*180/pi)
            # print("alpha3: ", alpha3*180/pi)
            s1 = alpha2
            s2 = alpha3
            const_ = 2048/pi
            # alpha1 = alpha1*self.__convert_const+self.__const1
            # alpha2 = alpha2*self.__convert_const+self.__const2
            # alpha3 = (alpha3-pi/2)*self.__convert_const+self.__const3
            # alpha4 = pitch*self.__convert_const_AX+self.__const4
            q = (alpha1,alpha2,alpha3-pi/2,pitch)
            # for i in range(0,len(q),1):
              # print("alpha"+str(i+1)+": "+str(q[i]))
            return True,q
        except Exception as e:
            print e
            return False, (0,0,0,0)
