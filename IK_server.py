#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
    	# DH pars (alpha, a, d, q):
    	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 =symbols('alpha0:7')
    	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

	# Create Modified DH parameters
    	# DH table
	s = { 	alpha0:	0,	a0:	0,	d1:	0.75,	q1:	q1,
		alpha1:	-pi/2,	a1:	0.35,	d2:	0,	q2:	-pi/2 + q2,
		alpha2:	0,	a2:	1.25,	d3:	0,	q3:	q3,
		alpha3:	-pi/2,	a3:	-0.054,	d4:	1.5,	q4:	q4,
		alpha4:	pi/2,	a4:	0,	d5:	0,	q5:	q5,
		alpha5:	-pi/2,	a5:	0,	d6:	0,	q6:	q6,
		alpha6:	0,	a6:	0,	d7:	0.303,	q7:	0}  
         
	# Define Modified DH Transformation matrix
    	# Transformation matrix
	def transform_matrix(alpha, a, d, q):
		tm = Matrix([[	cos(q),		-sin(q),		0,		a],
			[sin(q)*cos(alpha),	cos(q)*cos(alpha),	-sin(alpha),	-sin(alpha)*d],
			[sin(q)*sin(alpha),	cos(q)*sin(alpha),	cos(alpha),	cos(alpha)*d],
			[		0,		0,		0,		1]])
		return tm

	# Create individual transformation matrices
    	# Transformation between adjacent joints
    	T0_1 = transform_matrix(alpha0, a0, d1, q1).subs(s)
    	T1_2 = transform_matrix(alpha1, a1, d2, q2).subs(s)
    	T2_3 = transform_matrix(alpha2, a2, d3, q3).subs(s)
    	T3_4 = transform_matrix(alpha3, a3, d4, q4).subs(s)
    	T4_5 = transform_matrix(alpha4, a4, d5, q5).subs(s)
    	T5_6 = transform_matrix(alpha5, a5, d6, q6).subs(s)
    	T6_EE = transform_matrix(alpha6, a6, d7, q7).subs(s)

    	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

	# Extract rotation matrices from the transformation matrices
	# Define roll, pitch yaw
	r, p, y = symbols('r p y')
	R_x = Matrix([[1,		0,		0],
			[0,		cos(r),		-sin(r)],
			[0,		sin(r),		cos(r)]])
	R_y = Matrix([[cos(p),	0,		sin(p)],
			[0,		1,		0],
			[-sin(p),	0,		cos(p)]])
	R_z = Matrix([[cos(y),	-sin(y),	0],
			[sin(y),	cos(y),		0],
			[0,		0,		1]])
	R_EE_sym = R_z * R_y * R_x
	
        ###
	# Define some constants
	dtr = pi / 180
        rtd = 180 / pi
	WCtEE = 0.303	# Wrist-center to EE distance
	R_corr = R_z.subs(y, 180 * dtr) * R_y.subs(p, -90 * dtr)	# EE x-y-z to URDF x-y-z

	R_EE_sym = R_EE_sym * R_corr

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
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    # Wrist center postion
	    
	    R_EE = R_EE_sym.subs({'r': roll, 'p': pitch, 'y': yaw})

	    EE0 = Matrix([[px],
		          [py],
		          [pz]])
	   
	    WC0 = EE0 - WCtEE * R_EE[:,2]

	    # Calculate joint angles using Geometric IK method
	    theta1 = atan2(WC0[1], WC0[0])

	    # Triangle formed by joint2 - joint3 -joint5 (side_23, side_35, side_52)
	    # z-increment of joint 3
	    side_23 = 1.25
	    # distance between joint 3 and joint 5
	    side_35 = sqrt(0.54*0.54 + 0.96*0.96 + 0.054*0.054 +2*0.54*0.96)
	    # wrist center to origin
	    side_52 = sqrt(pow((sqrt(WC0[0] * WC0[0] + WC0[1] * WC0[1]) - 0.35), 2) + pow((WC0[2] - 0.75),2))
	    # angle caculation of triangle 2-3-5
	    angle_5 = acos((side_35 * side_35 + side_52 * side_52 - side_23 * side_23) / (2 * side_35 *side_52))
	    angle_2 = acos((side_23 * side_23 + side_52 * side_52 - side_35 * side_35) / (2 * side_23 *side_52))
	    angle_3 = acos((side_35 * side_35 + side_23 * side_23 - side_52 * side_52) / (2 * side_35 *side_23))
	    # theta2 is the angle away from x2
	    theta2 = pi/2 - angle_2 - atan2(WC0[2] - 0.75, sqrt(WC0[0]*WC0[0] + WC0[1]*WC0[1]) - 0.35)
	    # theta3 is the angle away from x3
	    theta3 = pi/2 - (angle_3 + atan2(0.054,1.5))

	    # composition of rotational transform from base-link to joint 3
	    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
	    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
	    # composition of rotations by joint 4,5,6 calculated from EE rotation and inverse of 0-3 rotation
	    R3_6 = R0_3.inv("LU") * R_EE

	    # Wrist twist angles theta4,5,6
	    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
	    theta6 = atan2(-R3_6[1,1], R3_6[1,0])	
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
    IK_server(
)
