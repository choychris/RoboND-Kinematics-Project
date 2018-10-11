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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        dtr = pi/180
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	    # Create Modified DH parameters
        s = {alpha0: 0,       a0: 0,      d1: 0.33+0.42,  q1: q1,
             alpha1: -90*dtr, a1: 0.35,   d2: 0,          q2: q2-90*dtr,
             alpha2: 0,       a2: 1.25,   d3: 0,          q3: q3,
             alpha3: -90*dtr, a3: -0.054, d4: 0.96+0.54,  q4: q4,
             alpha4: 90*dtr,  a4: 0,      d5: 0,          q5: q5,
             alpha5: -90*dtr, a5: 0,      d6: 0,          q6: q6,
             alpha6: 0,       a6: 0,      d7: 0.193+0.11, q7: 0}
        # Define Modified DH Transformation matrix
        def transMat(q, d, a, alpha):
            transform = Matrix([
                [cos(q), -sin(q), 0, a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
                [0, 0, 0, 1]])
            return transform.subs(s)
        T0_1 = transMat(q1, d1, a0, alpha0)
        # print (T0_1)
        T1_2 = transMat(q2, d2, a1, alpha1)
        # print (T1_2)
        T2_3 = transMat(q3, d3, a2, alpha2)
        # print (T2_3)
        T3_4 = transMat(q4, d4, a3, alpha3)
        # print (T3_4)
        T4_5 = transMat(q5, d5, a4, alpha4)
        # print (T4_5)
        T5_6 = transMat(q6, d6, a5, alpha5)
        # print (T5_6)
        T6_EE = transMat(q7, d7, a6, alpha6)
        # print (T6_EE)

        T0_2 = simplify(T0_1 * T1_2)
        T0_3 = simplify(T0_2 * T2_3)
        T0_4 = simplify(T0_3 * T3_4)
        T0_5 = simplify(T0_4 * T4_5)
        T0_6 = simplify(T0_5 * T5_6)
        T0_EE = simplify(T0_6 * T6_EE)
    
	    # Create individual transformation matrices
        def Rot_z(z):
            r = Matrix([[cos(z), -sin(z), 0, 0],
                        [sin(z),  cos(z), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
            return r
        def Rot_y(y):
            r = Matrix([[cos(y), 0,  sin(y), 0],
                        [0, 1, 0, 0],
                        [-sin(y), 0, cos(y), 0],
                        [0, 0, 0, 1]])
            return r
        def Rot_x(x):
            r = Matrix([[1, 0, 0, 0],
                        [0, cos(x), -sin(x), 0],
                        [0, sin(x),  cos(x), 0],
                        [0, 0, 0, 1]])
            return r
        R_y = Rot_y(-90*dtr)
        R_z = Rot_z(180*dtr)
        R_corr = simplify(R_z * R_y)
        T_final = simplify(T0_EE * R_corr)
        # Extract rotation matrices from the transformation matrices
        # R_final = T_final.row_del(3).col_del(3)
	
        ###

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
            # Create individual transformation matrices
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            # we need to inverse R_crr here since Rgazebo_EE = DH0_EE * R_corr
            # Rrpy(orientation from gazebo) = T0_EE * R_corr
            # T0_EE = Rrpy * R_corr.inv()
            R0_6 = simplify(Rot_z(yaw) * Rot_y(pitch) * Rot_x(roll) * R_corr.inv("LU"))
            nx = R0_6[0, 2]
            ny = R0_6[1, 2]
            nz = R0_6[2, 2]
            wx = px - 0.303 * nx
            wy = py - 0.303 * ny
            wz = pz - 0.303 * nz

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(wy, wx)
            d4 = 0.96+0.54
            d1 = 0.75
            a3 = 0.054
            a2 = 1.25
            a1 = 0.35
            diagonal = sqrt(wx**2 + wy**2) - a1 #line projected from straight distance b/w O2 & WC to plane x y 
            length_a = sqrt(a3**2 + d4**2)
            # here we need to subtract the length of a1 and d1 to obtain the actual length starting from O2 
            length_b = sqrt(diagonal**2 + (wz-d1)**2) # this is where I did wrong
            length_c = a2
            # Cosine Laws formula:
            # length_a**2 = length_b**2 + length_c**2 - 2*length_b*length_c*cos(angle_a)
            # inverse to find angle_a
            angle_a = acos((length_b**2+length_c**2-length_a**2)/(2*length_b*length_c))
            angle_b = acos((length_a**2+length_c**2-length_b**2)/(2*length_a*length_c))
            angle_c = acos((length_a**2+length_b**2-length_c**2)/(2*length_a*length_b))
            # print (angle_a, angle_b, angle_c)
            # print (angle_a+angle_b+angle_c)
            theta2 = pi/2 - angle_a - atan2((wz-d1), diagonal)
            theta3 = pi/2 - angle_b - atan2(a3, d4)

            # Calculate R4_6 
            # R0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            # R0_3 = R0_3.col_del(3).row_del(3)
            # R0_3 = R0_3.col_insert(3, Matrix([0, 0, 0]))
            # R0_3 = R0_3.row_insert(3, Matrix([[0, 0, 0, 1]]))
            R0_3 = T0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})[0:3, 0:3]
            R0_3 = R0_3.row_join(Matrix([[0], [0], [0]])).col_join(Matrix([[0, 0, 0, 1]]))
            R3_6 = simplify(R0_3.inv('LU') * R0_6)
            # r31 = R3_6[2, 0]
            # r32 = R3_6[2, 1]
            # r33 = R3_6[2, 2]
            # r11 = R3_6[0, 0]
            # r21 = R3_6[1, 0]
            # theta4 = atan2(r32, r33) #R3_4 #roll #Rx
            # theta5 = atan2(-r31, sqrt(r11**2+r21**2)) #R4_5 #pitch #Ry
            # theta6 = atan2(r21, r11) #R5_6 #yaw #Rz
            theta5 = atan2(sqrt(R3_6[0,2]**2+R3_6[2,2]**2), R3_6[1,2])
            if sin(theta5) < 0:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
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
    IK_server()
