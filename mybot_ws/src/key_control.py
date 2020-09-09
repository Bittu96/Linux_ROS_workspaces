#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from math import sin,cos,atan2,sqrt,fabs


#Define a RRBot joint positions publisher for joint controllers.
def bb_publisher():

	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('bb_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('linear/x', Twist, queue_size=10)
	
	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():

		#Have each joint follow a sine movement of sin(i/100).

                pub1.publish([1,0,0],[0,0,1])

		rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: bb_publisher()
	except rospy.ROSInterruptException: pass
