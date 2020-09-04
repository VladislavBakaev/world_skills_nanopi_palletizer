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
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

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
            self.Z_MAX = data['height_cap_before']
            self.Z_MIN = data['height_cap']
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
                    self.move_robot.moveToPoint([220,0,230,0])
                    #print("start")
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
                if msg[0]=='g':
                    list_ = msg.split(":")
                    if(not float(list_[2]) == 0):
                        self.move_robot.relaxManipulator(False)
                        #print(list_)
                        r = float(list_[2])
                        angle = (float(list_[1])-180.0)/180.0*math.pi
                        gripper_angle = float(list_[3])/180.0*math.pi
                        z = self.Z_MAX
                        if bool(int(list_[4])):
                            z = self.Z_MAX
                        else:
                            z = self.Z_MIN
                        #print(r, angle, gripper_angle, z)
                        self.move_robot.moveToPoint([r*math.cos(angle), r*math.sin(angle), z],gripper_angle)
                        #self.move_robot.gripper(list_[5][0])
                        if(not bool(int(list_[4]))):
                            self.move_robot.gripper(list_[5][0])
                if msg[0]=='1':
                    self.move_robot.relaxManipulator(False)
                    self.move_robot.moveToPoint([320,0,300],0)
                    self.move_robot.gripper(1)
                if msg[0]=='2':
                    self.move_robot.relaxManipulator(False)
                    r = 200
                    angle = (45.0)/180.0*math.pi
                    gripper_angle = 0
                    z = self.Z_MIN
                    self.move_robot.moveToPoint([r*math.cos(angle), r*math.sin(angle), z], gripper_angle)
                    self.move_robot.gripper(1)
                if msg[0]=='3':
                    self.move_robot.relaxManipulator(True)
                if msg[0]=='r':
                    self.sendFlag = True
                if msg[0]=='s':
                    self.sendFlag = False    

    def printParams(self, threadName, delay):
        global status  
        while True:
            if self.sendFlag:
                feedback_str = "g:"+status
                self.sendMessage(feedback_str)
            time.sleep(delay)

class MoveRobot(object):
    def __init__(self):
        self.client_point = rospy.ServiceProxy('/cmd_point', point_cmd)
        self.client_gripper = rospy.ServiceProxy('/gripper_cmd',point_cmd)
        rospy.Subscriber("/arm_joint_states", JointState,self.updateJointState)
        rospy.Subscriber('/temp_load',JointState,self.updateLoadTemp)
        self.stop = rospy.Publisher("stop_robot",Bool,queue_size=10)
        self.disable_tor = rospy.Publisher('disable_torque',Bool,queue_size = 10)
        self.gripper_open = 40
        self.gripper_close = 0
        self.current_joint_state = [0,0,0,0,0,0]
        self.current_temp = [0,0,0,0,0,0]
        self.current_load = [0,0,0,0,0,0]
        self.name_link = ['ang_joint_1','ang_joint_2','ang_joint_3','ang_joint_4','ang_joint_5','gripper']

    def gripper(self, flag):
        flag = int(flag)
        if(flag == 0):
            self.client_gripper(str(self.gripper_open))
        else:
            self.client_gripper(str(self.gripper_close))
        time.sleep(1)

    def compress(self,flag,port):
        flag = int(flag)
        start = '1'.encode()
        stop = '2'.encode()
        if(flag == 1):
            port.write(start)
        else:
            port.write(stop)
        time.sleep(1)

    def moveToPoint(self, point,roll):
        global status
	start_pitch = -1.57
        step = 0.05
        status = '1'
        while True:
            if(start_pitch > 0.7):
                break
            strPoint = list(map(str,point))
            strCmd = ' '.join(strPoint)+' '+str(start_pitch) +' ' +str(roll)
            result = self.client_point(strCmd)
            if(result.result):
                break
            else:
                start_pitch += step
        status = '0'
    
    def relaxManipulator(self,flag):
        mes = Bool()
        if(flag):
            mes.data = True
        else:
            mes.data = False
        self.disable_tor.publish(mes)
    def stopMove(self,flag):
        mes = Bool()
        i = 0
        mes.data = flag
        while i<15:
            self.stop.publish(mes)
            i +=1

    def updateJointState(self,msg):
        self.current_joint_state = msg.position
    
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
    config_file = os.path.join(THIS_FOLDER, 'config_ang.json')

    if not upd_motorcortex.loadParam(config_file):
        #print("Can't read config file")
        return

    upd_motorcortex.openUpd()

    upd_motorcortex.mainLoop()

if __name__ == '__main__':
    main()



