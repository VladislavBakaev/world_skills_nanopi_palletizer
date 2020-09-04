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
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32

status = '0'

class Udp2Motrorcortex(object):
    def __init__(self,robot):
        self.UDP_PORT_IN = 9090 # UDP server port
        self.UDP_PORT_OUT = 9090
        self.UDP_SERVER_ADDRESS = '127.0.0.1'
        self.MAX_UDP_PACKET=512 # max size of incoming packet to avoid sending too much data to the micro
        self.BROADCAST_MODE=False #set to True if you want a broadcast replay instead of unicast
        self.IP = 0
        self.sendFlag = True
        self.move_robot = robot      

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

        while True:
            (rlist, wlist, xlist) = select.select([self.udp_socket], [], [])           
            if self.udp_socket in rlist:
                    udp_data,udp_client = self.udp_socket.recvfrom(self.MAX_UDP_PACKET)
                    self.messageHandler(udp_data.decode("utf-8"))
                    time.sleep(0.001)

    def sendMessage(self, msg):
        if self.BROADCAST_MODE:
                self.udp_socket.sendto(msg.encode(),udp_broadcast)
        else:
                self.udp_socket.sendto(msg.encode(),(self.UDP_SERVER_ADDRESS, self.UDP_PORT_OUT))   

    def messageHandler(self, msg):
        if not msg==None:
            print(msg)
            list_ = msg.split(":")
            #print (list_,list_[1])
            if msg[0]=='t':
                self.move_robot.moveToPoint(int(list_[1][:-1]))
            if msg[0]=='r':
                self.sendFlag = True
            if msg[0]=='s':
                self.sendFlag = False

    def printParams(self, threadName, delay):
        global status  
        while True:
            if self.sendFlag:
                feedback_str = "b:"+status
                self.sendMessage(feedback_str)
            time.sleep(delay)

class MoveRobot(object):
    def __init__(self):
        rospy.Subscriber("/joint_state", JointState,self.updateJointState)
        self.target_point = rospy.Publisher('/point_id',Int32,queue_size = 10)
        self.current_joint_state = [0,0,0,0]
        self.current_velocity = [0,0,0,0]
        self.current_acc = [0,0,0,0]

    def moveToPoint(self, point):
        global status
        status = '1'
        for i in range(20):
            self.target_point.publish(point)
        rospy.sleep(0.5)
        while(math.fabs(self.current_velocity[0])>0.1 and math.fabs(self.current_velocity[1])>0.1 and math.fabs(self.current_velocity[2])>0.1 and math.fabs(self.current_velocity[3])>0.1):
              rospy.sleep(0.1)
        status = '0'

    def updateJointState(self,msg):
        self.current_joint_state = msg.position
        self.current_velocity = msg.velocity
        self.current_acc = msg.effort
    
def printMessage(msg):
    print(msg)

def main():
    rospy.init_node('udp_node')

    robot = MoveRobot()
    upd_motorcortex = Udp2Motrorcortex(robot)

    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(THIS_FOLDER, 'config_car.json')

    if not upd_motorcortex.loadParam(config_file):
        print("Can't read config file")
        return

    upd_motorcortex.openUpd()

    upd_motorcortex.mainLoop()

if __name__ == '__main__':
    main()
