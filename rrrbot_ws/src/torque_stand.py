#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math
from math import *
import numpy
from numpy import *
import tinyik

SimulationRunning = True

theta1i = 0#pi/2 + pi/3 #radians
theta2i = 0#0 + pi/3

theta1di = 0 #radians/sec
theta2di = 0

theta1ddi = 0 #radians/sec2
theta2ddi = 0

#\\\\\\\\\\\\\  PARAMETERS \\\\\\\\\\\\\\\\
m1 = 1      #kg
m2 = 1      #kg
l1 = 1    #m
l2 = 1    #m
g  = 9.81   #m/s2
#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

def torqueCalc(theta1,theta2,theta1d,theta2d,theta1dd,theta2dd):
  qdd = mat([ [theta1dd], [theta2dd] ])

  #INERTIAL TERM
  I11 = (m1 + m2)*l2**2 + m2*(l2**2) + 2*m2*l1*l2*cos(theta2)
  I12 = m2*(l2**2) + m2*l1*l2*cos(theta2) 
  I21 = m2*(l2**2) + m2*l1*l2*cos(theta2) 
  I22 = m2*(l2**2) 
  I = mat([[ I11, I12 ], [ I21, I22 ]])
  #print I

  #CORIOLIS TERM
  H1 = -m2*l1*l2*(2*theta1d*theta2d + theta2d**2)*sin(theta2)
  H2 = -m2*l2*l2*theta1d*theta2d*sin(theta2)
  H = mat([ [ H1 ], [ H2 ] ])
  #print H

  #GRAVITY TERM
  G1 = -(m1+m2)*g*l1*sin(theta1) - m2*g*l2*sin(theta1+theta2)
  G2 = -m2*g*l2*sin(theta1 + theta2) 
  G =mat([ [ G1 ], [ G2 ] ])
  #print G

  #JOINT TORQUES
  T = I*qdd + H + G 
  print "\nJoint1_torque :",T[0],"N/m"
  print "\nJoint2_torque :",T[1],"N/m"
  return T


arm = tinyik.Actuator([
'z', [l1, 0.0, 0.0], 
'z', [l2, 0.0, 0.0],
])

def rrbot_joint_torques_publisher():
	rospy.init_node('joint_torques_node', anonymous=True)
	pub1 = rospy.Publisher('/rrbot/joint1_torque_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/rrbot/joint2_torque_controller/command', Float64, queue_size=10)
	rate = rospy.Rate(100) #100 Hz
	time = 0
	theta1i = 0#pi/2 + pi/3 #radians
	theta2i = 0#0 + pi/3

	theta1di = 0 #radians/sec
	theta2di = 0

	theta1ddi = 0 #radians/sec2
	theta2ddi = 0
	while not rospy.is_shutdown():
		  arm.ee = [0.1*sin(time),0.5,1.0] 
		  #print arm.angles

		  theta1_track = arm.angles[0]
		  theta2_track = arm.angles[1]
		  
		  theta1 = theta1_track
		  theta2 = theta2_track
		  
		  theta1d = -theta1i + theta1
		  theta2d = -theta2i + theta2

		  theta1i = theta1
		  theta2i = theta2

		  theta1dd = -theta1di + theta1d
		  theta2dd = -theta2di + theta2d

		  theta1di = theta1d
		  theta2di = theta2d

		  T = torqueCalc(theta1,theta2,theta1d,theta2d,theta1dd,theta2dd)
		  #print ravel(T)[0]
		  time += 0.01
               	  pub1.publish(ravel(T)[0])
		  pub2.publish(ravel(T)[1])
		  rate.sleep() #sleep for rest of rospy.Rate(100)

if __name__ == '__main__':
	try: rrbot_joint_torques_publisher()
	except rospy.ROSInterruptException: pass
