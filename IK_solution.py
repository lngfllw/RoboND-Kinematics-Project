# -*- coding: utf-8 -*-
"""
Created on Sun Jan 27 18:04:33 2019

@author: Engineering
"""

#Inverse Kinematics Solution Code

#Define DH param symbols 
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angle

# Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') 

# info from KR2010 kinematics
# modified DH table
DHT = {	alpha0:     0, a0: 	    0,     d1:  0.75, q1:     q1,
			alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2, #account for 90 deg diff
		    alpha2:     0, a2:   1.25, d3:     0, q3:     q3,
		    alpha3: -pi/2, a3: -0.054, d4:   1.5, q4:     q4,
		    alpha4:  pi/2, a4:      0, d5:     0, q5:     q5,
		    alpha5: -pi/2, a5:      0, d6:     0, q6:     q6,
		    alpha6:     0, a6:      0, d7: 0.303, q7:      0}

def TF_mat(alpha, a, d, q):
    TF = Matrix([[          cos(q),            -sin(),           0,             a],
	           [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
	           [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
	           [                 0,                 0,           0,             1]])
    return TF

# create transofrm mats
T0_1 = TF_mat(alpha0, a0, d1, q1).subs(DHT)
T1_2 = TF_mat(alpha1, a1, d2, q2).subs(DHT)
T2_3 = TF_mat(alpha2, a2, d3, q3).subs(DHT)
T3_4 = TF_mat(alpha3, a3, d4, q4).subs(DHT)
T4_5 = TF_mat(alpha4, a4, d5, q5).subs(DHT)
T5_6 = TF_mat(alpha5, a5, d6, q6).subs(DHT)
T6_G = TF_mat(alpha6, a6, d7, q7).subs(DHT)

T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

# Extract end-effector position and orientation from request
# px,py,pz = end-effector position
# roll, pitch, yaw = end-effector orientation
px = req.poses[x].position.x
py = req.poses[x].position.y
pz = req.poses[x].position.z

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])
    
#find gripper rotation mat
#roll pitch yaw rot mat

r, p, y = symbols('r p y')

R_x = Matrix([[ 1,       0,       0],
  			[   0,  cos(r), -sin(r)],
  			[   0,  sin(r),  cos(r)]])
    
R_y = Matrix([[ cos(p),        0,  sin(p)],
  			[        0,        1,           0],
  			[  -sin(p),        0,  cos(p)]])
    
R_z = Matrix([[ cos(y), -sin(y),        0],
  			[   sin(y),  cos(y),        0],
  			[        0,         0,        1]])
    
# correct from Kuka kinematics
R_corr = R_z.subs(y, pi/2.) * R_y.subs(p, -pi/4.)
    
R_G = R_z * R_y * R_x * R_corr

# position of gripper and wrist center
G_pos = Matrix([[px], [py], [pz]])
WC_pos = G_pos - (.303) * R_G[:,2]


theta1 = atan2(WC_pos[1], WC_pos[0])

# SSS triangle 
side_a = 1.5 #1.501?
side_b = sqrt((sqrt(WC_pos[0]**2 + WC_pos[1]**2) - 0.35)**2 + (WC_pos[2] - .75)**2)
side_c = 1.25

angle_a = acos((side_b**2 + side_c**2 - side_a**2) / (2 * side_b * side_c))
angle_b = acos((side_a**2 + side_c**2 - side_b**2) / (2 * side_a * side_c))
# DONT NEED angle_c = acos((side_a**2 + side_b**2 - side_c**2) / (2 * side_a * side_b))

theta2 = pi/2 - angle_a - atan2(WC_pos[2] - 0.75, sqrt(WC_pos[0]**2 + WC_pos[1]**2) - 0.35)
theta3 = pi/2 - (angle_b + 0.036)#.036 accounts for sag in link 4 of -.054 ?

R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

R3_6 = R0_3.inv("LU") * R_G

# Euler angles from rotation matrix
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])





