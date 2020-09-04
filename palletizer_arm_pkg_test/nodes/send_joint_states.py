#!/usr/bin/env python
import rospy  
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64 
import math

joint_pub = rospy.Publisher("/palletizer_robot/joint_states_remap", JointState, queue_size=10)
fromEncToRadFor1066428 = 4096/(2*math.pi)
fromEncToRadFor12 = (1024*180)/(300*math.pi)
minEncPoseForGripper = 390
fromEncToLinGripper = 7.75
scale = 100
zeroPose = [2048,1930,2230,512,512]

def app_new_joint_state(joint_state_msg,joint_name,position,zero):
    joint_state_msg.name.append(joint_name)
    if (joint_name == 'pal_joint_3'):
        position = (position-zero)/fromEncToRadFor12
    else:
        position = (position-zero)/fromEncToRadFor1066428
    joint_state_msg.position.append(round(position,2))
    
    return joint_state_msg

def read_pose_callback(data_list):
    joint_state = JointState()

    for i in range(4):
        joint_state = app_new_joint_state(joint_state,'pal_joint_'+str(i),data_list.position[i],zeroPose[i])
    joint_state = app_new_joint_state(joint_state,'gripper',data_list.position[4],zeroPose[4])


    joint_state.header.stamp = rospy.Time.now()
    joint_pub.publish(joint_state)

if __name__ == '__main__':
    rospy.init_node('Send_joint_state')
    rospy.loginfo('Send_joint_state node was started')
    rospy.Subscriber('/arm_joint_states', JointState, read_pose_callback)
    rospy.spin()
