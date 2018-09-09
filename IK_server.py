#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import math as m
import numpy as np
# This function define transformation matrix according to DH parameters
def Transformation_matrix(alpha,a,d,q):
    T =  Matrix([[cos(q),            -sin(q),            0,              a         ],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                 [ 0,                 0,                 0,            1           ]])
    return T


### FK code here : Performed as global variables for execution speed up 
# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	
# Create Modified DH parameters
s = {alpha0: 0,     a0: 0,      d1: 0.75,  q1:q1,
     alpha1: -pi/2, a1: 0.35,   d2: 0,     q2:-pi/2.+q2,
     alpha2: 0,     a2: 1.25,   d3: 0,     q3:q3,
     alpha3: -pi/2, a3: -0.054, d4: 1.5,   q4:q4,
     alpha4: pi/2,  a4: 0,      d5: 0,     q5:q5,
     alpha5: -pi/2, a5: 0,      d6: 0,     q6:q6,
     alpha6: 0,     a6: 0,      d7: 0.303, q7:0}
	
# Defie Modified DH Transformation matrix
T0_1 = Transformation_matrix(alpha0,a0,d1,q1)
T0_1 = T0_1.subs(s)

T1_2 = Transformation_matrix(alpha1,a1,d2,q2)
T1_2 = T1_2.subs(s)
T2_3 = Transformation_matrix(alpha2,a2,d3,q3)
T2_3 = T2_3.subs(s)
T3_4 = Transformation_matrix(alpha3,a3,d4,q4)
T3_4 = T3_4.subs(s)
T4_5 = Transformation_matrix(alpha4,a4,d5,q5)
T4_5 = T4_5.subs(s)
T5_6 = Transformation_matrix(alpha5,a5,d6,q6)
T5_6 = T5_6.subs(s)
T6_G = Transformation_matrix(alpha6,a6,d7,q7)
T6_G = T6_G.subs(s)
# Create total transformation matrices
T0_3 = T0_1*T1_2*T2_3 
T_total = T0_3*T3_4*T4_5*T5_6*T6_G 
# Extract rotation matrices from the transformation matrices
# This step is needed in the loop of IK, but performed out of the loop for optimization 	
R0_3 = T0_3[0:3,0:3]
# we use lambdify to convert sympy names to numerical library : numpy
# used for fast evaluation of expressions (It saves alot of time)
# The time required for calculating IK using this method is about 1/100 of the time required if we not use it  
R03 = lambdify((q1,q2,q3),R0_3,"numpy") 
###

# rpy2rot_DH function
#This function takes roll, pitch and yaw angles as arguments and returns the corresponding rotation matrix
#The tricky point that the relation between DH parameters and urdf parameters are change in axes
#Xg in DH equivalent to Zg in urdf, Zg equivalent to Xg, and Yg eqivalent to -Yg 
#We use this note and change columns of the result rotation matrix to match DH parameters directly 
def rpy2rot_DH(r,p,y):
    """
    :param r: roll angle
    :type r: float
    :param p: pitch angle 
    :type r: float
    :param y: yaw angle 
    :type c: float 
    :return: Rotation matrix in DH prameters 
    :retype: Matrix
    """
    Rrpy_modified = Matrix([[cos(y)*sin(p)*cos(r)+sin(y)*sin(r), -cos(y)*sin(p)*sin(r)+sin(y)*sin(r), cos(y)*cos(p)],
	 	            [sin(y)*sin(p)*cos(r)-cos(y)*sin(r), -sin(y)*sin(p)*sin(r)-cos(y)*cos(r), sin(y)*cos(p)],
		            [cos(p)*cos(r),                      -cos(p)*sin(r),                      -sin(p)]])
    return Rrpy_modified

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            ### Your IK code here
	    # Compute R0_6 w.r.t DH parameters directly from roll,pitch, and yaw angle using the relation between DH and urfd parameters 
	    R0_6 = rpy2rot_DH(roll,pitch,yaw)
	    # calculate wrist center 
	    wcx = px - 0.303*R0_6[0,2]
	    wcy = py - 0.303*R0_6[1,2]
	    wcz = pz - 0.303*R0_6[2,2] 
	    # Calculate joint angles using Geometric IK method
	    # from geometry of the kuka arm we calculate the first three joint variables
	    theta1 = m.atan2(wcy,wcx)
	    # Theta 2 and Theta 3 require some geometry (The geoemtry explained in the lesson)
    	    l = m.sqrt(wcx**2+wcy**2)-0.35
    	    ca = (1.25**2+l**2+(wcz-0.75)**2-1.5**2-0.054**2)/(2*1.25*m.sqrt(l**2+(wcz-0.75)**2))
    	    a = m.acos(ca)
	    theta2 = m.pi/2 - a - m.atan2(wcz-0.75,l) 
            cb = (1.25**2+1.5**2+0.054**2-l**2-(wcz-0.75)**2)/(2*1.25*m.sqrt(1.5**2+0.054**2))
            b = m.acos(cb)
            theta3 = m.pi/2 - b - m.atan2(0.054,1.5)
            #Complete the orientation part R3_6
	    iR0_3 = np.asmatrix(R03(theta1,theta2,theta3))
	    R3_6 = iR0_3.I*R0_6
	    #calculate the last three joint variables of the spherical wrist 
	    theta4 = m.atan2(R3_6[2,2],-R3_6[0,2])
   	    theta5 = m.atan2(m.sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta6 = m.atan2(-R3_6[1,1],R3_6[1,0])
	    ###
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
