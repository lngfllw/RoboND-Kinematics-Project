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
		#using J Craig DH param convention
        q, d, a, alpha = symbols('q, d, a, alpha')#('q', 'd', 'a', 'alpha')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i = angle x_i+1 - x_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # dist x_i - x_i-1
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # dist z_i+1 - z_i
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #angle z_i+1 - z_i
	
	
		# Create Modified DH parameters
        s = {	alpha0:     0, a0: 	    0, d1:  0.75,
    			alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2, #account for 90 deg diff
			    alpha2:     0, a2:   1.25, d3:     0,
			    alpha3: -pi/2, a3: -0.054, d4:   1.5,
			    alpha4:  pi/2, a4:      0, d5:     0,
			    alpha5: -pi/2, a5:      0, d6:     0,
			    alpha6:     0, a6:      0, d7: 0.303, q7:      0}

		# Define Modified DH Transformation matrix
	
		# Create individual transformation matrices
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
		           [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
		           [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
		           [                   0,                   0,            0,               1]])
        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
				   [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
				   [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
				   [                   0,                   0,            0,               1]])
        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
				   [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
				   [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
				   [                   0,                   0,            0,               1]])
        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
				   [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
				   [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
				   [                   0,                   0,            0,               1]])
        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
				   [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
				   [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
				   [                   0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
				   [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
				   [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
				   [                   0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s)

        T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
				   [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
				   [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
				   [                   0,                   0,            0,               1]])
        T6_G = T6_G.subs(s)

		# Extract rotation matrices from the transformation matrices
        T0_2 = simplify(T0_1 * T1_2) # to link 2
        T0_3 = simplify(T0_2 * T2_3) # to link 3
        T0_4 = simplify(T0_3 * T3_4) # to link 4
        T0_5 = simplify(T0_4 * T4_5) # to link 5
        T0_6 = simplify(T0_5 * T5_6) # to link 6
        T0_G = simplify(T0_6 * T6_G) # to link 6

		# correct for the gripper difference in urdf / dh convention
        R_corr = ', Matrix([
		[                  1.0, -1.22464679914735e-16, -1.22464679914735e-16, 0],
		[-1.22464679914735e-16,                  -1.0,  1.49975978266186e-32, 0],
		[-1.22464679914735e-16,                     0,                  -1.0, 0],
		[                    0,                     0,                     0, 1]]))
        print('R_corr = ', R_corr)
		
		#grippy matreez rotangtions
        R0_3 = Matrix([[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
						[sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
						[        cos(q2 + q3),        -sin(q2 + q3),        0]])
						
				
		#apply correction to total transform
        T_total = simplify(T0_G * R_corr)
        print('T_total = ', T_total)
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
            n_x = T0_6[0,2]
            n_y = T0_6[1,2]
            n_z = T0_6[2,2]
			# Compensate for rotation discrepancy between DH parameters and Gazebo
            wx = px - (d6+d7)*n_x
            wy = py - (d6+d7)*n_y
            wz = pz - (d6+d7)*n_z
			#
			# Calculate joint angles using Geometric IK method
            theta1 = atan2(wy, wx)
	#	    theta2/3 - law of cosines
            wxy = sqrt(wx**2 + wy**2)
            B2 = (wxy - 0.35)**2 + (wz - .75)**2 #B squared
            thet_b = acos((a2**2 +d4**2 - B2)/(a2*d4))
            theta3 = np.pi/2 - thet_b
			
            thet_a = acos((a2**2 + B2 - d4**2)/(2*d4*sqrt(B2)))
            thet_2pa = atan2(wxy/wz)
            theta2 = thet_2pa - thet_a
			
			#R3_6 = inv(R0-3)*Rrpy
            R0_3ev = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
			
            R_x = Matrix([[ 1,          0,          0],
              			[   0,  cos(roll), -sin(roll)],
              			[   0,  sin(roll),  cos(roll)]])
            R_y = Matrix([[ cos(pitch),        0,  sin(pitch)],
              			[            0,        1,           0],
              			[  -sin(pitch),        0,  cos(pitch)]])
            R_z = Matrix([[ cos(yaw), -sin(yaw),        0],
              			[   sin(yaw),  cos(yaw),        0],
              			[           0,          0,        1]])
              			
            Rrpy = R_z * R_y * R_z * R_corr
            R3_6 = R0_3ev.inv("LU") * Rrpy
			
            theta5 = acos(R3_6[1,2])
            theta6 = acos(R3_6[1,0]/sin(theta5))
            theta4 = -acos(R3_6[0,2]/sin(theta5))
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
