#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import math
from inverse_problem_srv.srv import publish_cmd,publish_cmdResponse

jointPub = rospy.Publisher("/cmd_joints",JointState,queue_size = 100)

fromEncToRadFor1066428 = 4096/(2*math.pi)
fromRncToRadFor12 = (1024*180)/(300*math.pi)
minEncPoseForGripper = 390
fromEncToLinGripper = 7.75
zeroPose = [2048,1930,2230,512,512]

currentState = JointState()
currentState.position = [0,0,0,0]
maxVelocity = 1
kVelRadSToMotorVel = 0.11/30*math.pi
kAccRadToMotorAcc = 8.58/180*math.pi
kAcc = 0.7
timeRate = 20
countOfJoint = 4

def UpdateCurJointState(curJS):
    global currentState
    currentState.position = curJS.position

def reachingOfThePoint(currentStateList,goalStatelList,error):
    reachingPoint = False
    if(math.fabs(currentStateList[0]-goalStatelList[0])<error and
       math.fabs(currentStateList[1]-goalStatelList[1])<error and
       math.fabs(currentStateList[2]-goalStatelList[2])<error and
       math.fabs(currentStateList[3]-goalStatelList[3])<error):
       reachingPoint = True
    return reachingPoint

def way_compute(posList):
    global currentState
    wayList = list()
    for i in range(len(posList)):
        wayList.append(posList[i]-currentState.position[i])
    return wayList

def max_way(wayList):
    maxWay = wayList[len(wayList)-1]
    for i in range(len(wayList)-1):
        if (math.fabs(wayList[i])>math.fabs(maxWay)):
            maxWay = wayList[i]
        else:
            continue
    return maxWay

def parse_msg(msg):
    jointState = JointState()
    msgList = msg.jointState.split()
    jointState.name = msgList[0:countOfJoint]
    jointState.position = msgList[countOfJoint:]
    jointState.position = [float(el) for el in jointState.position] 
    return jointState

def convert_pose_vel_acc(name,poseList,velList,accList):
    jointcmd = JointState()
    jointcmd.name = name
    poseListPub = []
    velListPub = []
    accListPub = []
    for i in range(len(name)):
        velListPub.append(math.fabs(round(velList[i]/kVelRadSToMotorVel)))
        accListPub.append(math.fabs(round(accList[i]/kAccRadToMotorAcc)))
        if (accListPub[i]==0):
            accListPub[i] = 1
        if (velListPub[i]==0):
            velListPub[i] = 1
        if(name[i] == 'pal_joint_3'):
            poseListPub.append(round((poseList[i]*fromRncToRadFor12)+zeroPose[i]))
        elif(name[i] == 'gripper'):
            poseListPub.append(round(poseList[i]*fromEncToLinGripper + minEncPoseForGripper))
        else:
            poseListPub.append(round((poseList[i]*fromEncToRadFor1066428)+zeroPose[i]))
    jointcmd.position = poseListPub
    jointcmd.velocity = velListPub
    jointcmd.effort = accListPub

    return jointcmd    


def move_of_trapeze_principle(msg):
    msg = parse_msg(msg)
    velList = list()
    accList = list()
    wayList = way_compute(msg.position)
    maxWay = max_way(wayList)
    kVel = (1-kAcc)
    timeOfWay = math.fabs(maxWay/(maxVelocity*kVel))
    for i in range(len(msg.position)):
        velList.append(wayList[i]/(timeOfWay*kVel))
        accList.append(velList[i]/(timeOfWay*kAcc))
    jointCmd = convert_pose_vel_acc(msg.name,msg.position,velList,accList)

    rate = rospy.Rate(5)
    while(not reachingOfThePoint(currentState.position,msg.position,0.05)):
        jointCmd.header.stamp = rospy.Time.now()
        jointPub.publish(jointCmd)
        rate.sleep()
    return publish_cmdResponse(True)
     

if __name__=='__main__':
    rospy.init_node('convert_and_publish_state')
    rospy.loginfo('Start_convert_and_publish_state_node')
    rospy.Service('/cmd_joint_state_in_manip_coord', publish_cmd, move_of_trapeze_principle)
    rospy.Subscriber('/palletizer_robot/joint_states_remap',JointState,UpdateCurJointState)
    rospy.spin()
