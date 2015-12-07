#!/usr/bin/env python  
import roslib
import rospy
import tf
import sys
import math
import random

if __name__ == '__main__':

	if len(sys.argv) < 9:
		print ""
		print "Use python", sys.argv[0], "<parent-frame> <child-frame> <x:m> <y:m> <z:m> <roll:deg> <pitch:deg> <yaw:deg>"
		print ""

	else:
		rospy.init_node('SimpleTransformBroadcaster' + str(random.randint(0, 1000000)))
		print("Running...")

		br = tf.TransformBroadcaster()
		rate = rospy.Rate(10.0)
	
		parent_frame = sys.argv[1]
		child_frame = sys.argv[2]
		x = float(sys.argv[3])
		y = float(sys.argv[4])
		z = float(sys.argv[5])
		roll = (float(sys.argv[6]) / 180.0) * math.pi
		pitch = (float(sys.argv[7]) / 180.0) * math.pi
		yaw = (float(sys.argv[8]) / 180.0) * math.pi

		while not rospy.is_shutdown():

			# OBS: o formato da transformada em C/C++ 
			# eh diferente (os parametros estao em outra ordem)
			br.sendTransform((x, y, z), # t (x, y, z)
				tf.transformations.quaternion_from_euler(roll, pitch, yaw), # R (roll, pitch, yaw)
				rospy.Time.now(), # time
				child_frame, # child frame
				parent_frame) # parent frame

			rate.sleep()
