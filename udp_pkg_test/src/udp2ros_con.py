#!/usr/bin/env python
import rospy
import time
import socket,select
import time
import os
import json
import argparse
import sys
import math
import thread
import time
import serial
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from srv_msg.srv import DistCmd

status = '0'

class Udp2Motrorcortex(object):
    def __init__(self,robot):
        self.UDP_PORT_IN = 9090 # UDP server port
        self.UDP_PORT_OUT = 9090
        self.UDP_SERVER_ADDRESS = '127.0.0.1'
        self.MAX_UDP_PACKET=512 # max size of incoming packet to avoid sending too much data to the micro
        self.BROADCAST_MODE=False #set to True if you want a broadcast replay instead of unicast
        self.IP = 0 
        self.port = serial.Serial("/dev/ttyS1", baudrate=115200, timeout=1.0)
        self.sendFlag = True
        self.manipulatorStatus = 0
        self.move_robot = robot
        self.buttonStatus = False
        
        
    def loadParam(self, config_file):
        if config_file==None:
            return False
        with open(config_file) as json_file:
            data = json.load(json_file)
            self.UDP_PORT_IN = data['input_port']
            self.UDP_PORT_OUT = data['output_port']
            self.UDP_SERVER_ADDRESS = data['server_address']
        return True

    def openUpd(self):
        self.udp_client_address=(self.UDP_SERVER_ADDRESS, self.UDP_PORT_OUT) #where to forward packets coming from seriaal port
        self.udp_server_address = ('',self.UDP_PORT_IN) #udp server
        self.udp_broadcast=('<broadcast>',self.UDP_PORT_OUT) #broadcast address

        self.udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.udp_socket.bind(self.udp_server_address)
        if self.BROADCAST_MODE: 
            self.udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

    def mainLoop(self):
        thread.start_new_thread( self.printParams, ("Thread-1", 0.1, ))
        thread.start_new_thread( self.readStatus, ("Thread-2", 0.01, ))
        

        while True:
            (rlist, wlist, xlist) = select.select([self.udp_socket], [], [])           
            if self.udp_socket in rlist:
                    udp_data,udp_client = self.udp_socket.recvfrom(self.MAX_UDP_PACKET)
                    self.messageHandler(udp_data.decode("utf-8"))
                    time.sleep(0.001)


    def readStatus(self, threadName, delay):
        for i in range(25):
            self.port.readline()
        while True:
            result = ""
            while True:
                result_ = self.port.readline().decode()
                if not result_==None  and len(result_)>0:
                    result = result_
                    #print (result)
                    break
                    #print(len(result))

            if len(result)==11:
                if not self.buttonStatus:
                    self.move_robot.stopMove(True)
                    #print("stop")
                self.buttonStatus = True
            if len(result)==9:
                if self.buttonStatus:
                    self.move_robot.stopMove(False)
                self.buttonStatus = False

            #time.sleep(delay)

    def sendMessage(self, msg):
        if self.BROADCAST_MODE:
                self.udp_socket.sendto(msg.encode(),udp_broadcast)
        else:
                self.udp_socket.sendto(msg.encode(),(self.UDP_SERVER_ADDRESS, self.UDP_PORT_OUT))   

    def messageHandler(self, msg):
        if not msg==None:
            print(msg)
            if(self.buttonStatus == False):
                if msg[0]=='c':
                    list_ = msg.split(":")
                    #command
                    if(int(list_[1])==1):
                        self.move_robot.moveVel(float(list_[2]))
                        if(not float(list_[3][:-1])==0):
                            rospy.sleep(float(list_[3][:-1]))
                            self.move_robot.moveVel(0)
                    else:
                        self.move_robot.moveDistVel(float(list_[3][:-1]))
                        self.move_robot.moveDist(float(list_[2]))
                if msg[0]=='r':
                    self.sendFlag = True
                if msg[0]=='s':
                    self.sendFlag = False    

    def printParams(self, threadName, delay):
        global status  
        while True:
            if self.buttonStatus == True:
                status = '2'

            if self.sendFlag:
                feedback_str = "c:"+status
                self.sendMessage(feedback_str)
            time.sleep(delay)

class MoveRobot(object):
    def __init__(self):
        self.client_dist = rospy.ServiceProxy('/cmd_lin_dist', DistCmd)
        self.dist_vel_pub = rospy.Publisher('/cmd_dist_vel', Float64, queue_size = 10)
        self.vel_pub = rospy.Publisher("/cmd_vel",Float64,queue_size=10)
        rospy.Subscriber("/vel_joint_states", JointState,self.updateVelJoint)
        rospy.Subscriber('/temp_load',JointState,self.updateLoadTemp)
        self.stop = rospy.Publisher("stop_robot",Bool,queue_size=10)
        self.disable_tor = rospy.Publisher('disable_torque',Bool,queue_size = 10)
        self.current_joint_vel = [0]
        self.current_temp = [0]
        self.current_load = [0]

    def moveDist(self, dist):
        global status
        status = '1'
        self.client_dist(dist)
        status = '0'

    def moveDistVel(self, vel):
        for _ in range(15):
            self.dist_vel_pub.publish(vel)        

    def moveVel(self, vel):
        global status
        if(vel == 0.0):
            status = '0'
        else:
            status = '1'
        for _ in range(15):
            self.vel_pub.publish(vel)
    
    def stopMove(self,flag):
        mes = Bool()
        mes.data = flag
        for _ in range(15):
            self.stop.publish(mes)

    def updateVelJoint(self,msg):
        self.current_joint_vel = msg.velocity
    
    def updateLoadTemp(self,msg):
        self.current_temp = msg.position
        self.current_load = msg.velocity
    
def printMessage(msg):
    #print(msg)
    pass

def main():
    rospy.init_node('udp_node')

    robot = MoveRobot()
    upd_motorcortex = Udp2Motrorcortex(robot)

    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(THIS_FOLDER, 'config_con.json')

    if not upd_motorcortex.loadParam(config_file):
        #print("Can't read config file")
        return

    upd_motorcortex.openUpd()

    upd_motorcortex.mainLoop()

if __name__ == '__main__':
    main()




