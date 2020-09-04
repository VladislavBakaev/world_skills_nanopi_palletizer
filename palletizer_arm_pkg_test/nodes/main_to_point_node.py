#!/usr/bin/env python
import rospy  
import math
from RoboticArmPalletizerClass import RoboticArm
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from inverse_problem_srv.srv import publish_cmd

nameList = ['pal_joint_0','pal_joint_1','pal_joint_2','pal_joint_3']
jointStateSrv = rospy.ServiceProxy('/cmd_joint_state_in_manip_coord', publish_cmd)

def ParseMsg(msg):
    try:
        coord_list = msg.point.split()
        x = float(coord_list[0])
        y = float(coord_list[1])
        z = float(coord_list[2])
        pith = float(coord_list[3])
        return x,y,z,pith
    except ValueError:
        rospy.logerr('Input Error')

def MoveToPointCallback(msg):
    x,y,z,pitch = ParseMsg(msg)
    roboticArm = RoboticArm()
    availJointState,goalJointState = roboticArm.InversProblem(x,y,z,pitch)

    if (not availJointState):
       # rospy.loginfo('Point cannot be reached')
        return point_cmdResponse(False)
    else:
        #rospy.loginfo('Wait...')
        goalJointState = [str(el) for el in goalJointState]
        strName = ' '.join(nameList)
        strJS = ' '.join(goalJointState)
        strCmd = strName + ' ' + strJS
        result = jointStateSrv(strCmd)
        #rospy.loginfo('Well Done!!!')
    	return point_cmdResponse(True)

if __name__=='__main__':
    rospy.init_node('main_to_point_node')
    #rospy.loginfo('Main node for move to point was started')
    rospy.Service('/cmd_point', point_cmd, MoveToPointCallback)
    rospy.spin()
