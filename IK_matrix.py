# -*- coding: utf-8 -*-
"""
Created on Sun Jan 27 16:47:42 2019

@author: Engineering
"""

from sympy import *
from time import time
from mpmath import radians
#import tf
import numpy as np


    
########################################################################################
## 

## Insert IK code here!

q, d, a, alpha = symbols('q, d, a, alpha')#('q', 'd', 'a', 'alpha')
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint angle, theta_i = angle x_i+1 - x_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset, dist x_i - x_i-1
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link length, dist z_i+1 - z_i
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angle, angle z_i+1 - z_i


	# Create Modified DH parameters
s = {	alpha0:     0, a0: 	    0, d1:  0.75, q1:     q1,
			alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2, #account for 90 deg diff
		    alpha2:     0, a2:   1.25, d3:     0, q3:     q3,
		    alpha3: -pi/2, a3: -0.054, d4:   1.5, q4:     q4,
		    alpha4:  pi/2, a4:      0, d5:     0, q5:     q5,
		    alpha5: -pi/2, a5:      0, d6:     0, q6:     q6,
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
#   T0_2 = simplify(T0_1 * T1_2) # to link 2
#   T0_3 = simplify(T0_2 * T2_3) # to link 3
#   T0_4 = simplify(T0_3 * T3_4) # to link 4
#   T0_5 = simplify(T0_4 * T4_5) # to link 5
#   T0_6 = simplify(T0_5 * T5_6) # to link 6
#   T0_G = simplify(T0_6 * T6_G) # to link 6


T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

T0_3 = T0_1 * T1_2 * T2_3

R0_3 = T0_3[0:3,0:3]

R0_G = T0_G[0:3, 0:3]

R3_6 = R0_3.inv("LU") * R0_G


#euler angles alpha beta gamma 
#where alpha is rot(z), beta= rot(y), gamma = rot(x)
	
# Extract end-effector position and orientation from request
# px,py,pz = end-effector position
# roll, pitch, yaw = end-effector orientation
#px = req.poses[x].position.x
#py = req.poses[x].position.y
#pz = req.poses[x].position.z
#
#(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
#    [req.poses[x].orientation.x, req.poses[x].orientation.y,
#        req.poses[x].orientation.z, req.poses[x].orientation.w])
#
#### Your IK code here
#
#r,p,y = symbols('r', 'p', 'y')
#
#R_x = Matrix([[ 1,          0,          0],
#  			[   0,  cos(roll), -sin(roll)],
#  			[   0,  sin(roll),  cos(roll)]])
#R_y = Matrix([[ cos(pitch),        0,  sin(pitch)],
#  			[            0,        1,           0],
#  			[  -sin(pitch),        0,  cos(pitch)]])
#R_z = Matrix([[ cos(yaw), -sin(yaw),        0],
#  			[   sin(yaw),  cos(yaw),        0],
#  			[          0,         0,        1]])
#
#	
#R_G = R_z * R_y * R_x
#	# Compensate for rotation discrepancy between DH parameters and Gazebo
#R_corr = R_z.subs(yaw, pi) * R_y.subs(pitch, pi/2)
#R_G = R_G * R_corr
#
#	# calculate wrist center
#wx = px - (d6+d7)*R_G[0,2]
#wy = py - (d6+d7)*R_G[1,2]
#wz = pz - (d6+d7)*R_G[2,2]
#	#
#	# Calculate joint angles using Geometric IK method
#theta1 = atan2(wy, wx)
##	    theta2/3 - law of cosines
#wxy = sqrt(wx**2 + wy**2)
#B2 = (wxy - 0.35)**2 + (wz - .75)**2 #B squared
#thet_b = acos((a2**2 +d4**2 - B2)/(a2*d4))
#theta3 = np.pi/2 - thet_b
#	
#thet_a = acos((a2**2 + B2 - d4**2)/(2*d4*sqrt(B2)))
#thet_2pa = atan2(wxy,wz)
#theta2 = thet_2pa - thet_a
#	
#	#R3_6 = inv(R0-3)*Rrpy
#R0_3 = T0_1[0:3, 0:3] * T1_2[0:3 , 0:3] * T2_3[0:3, 0:3]
#R0_3ev = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
#	
#R3_6 = R0_3ev.inv("LU") * R_G
#	
##    theta5 = acos(R3_6[1,2])
##    theta6 = acos(R3_6[1,0]/sin(theta5))
##    theta4 = -acos(R3_6[0,2]/sin(theta5))
#theta4 
#
### 
#########################################################################################
#
#########################################################################################
### For additional debugging add your forward kinematics here. Use your previously calculated thetas
### as the input and output the position of your end effector as your_ee = [x,y,z]
#
### (OPTIONAL) YOUR CODE HERE!
#
### End your code input for forward kinematics here!
#########################################################################################
#
### For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
#   