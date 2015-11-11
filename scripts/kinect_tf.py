#!/usr/bin/env python  
import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('RosAriaLaserTf')
    print("Running...")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():

    	# OBS: o formato da transformada em C/C++ 
    	# eh diferente (os parametros estao em outra ordem)
        br.sendTransform((0, 1.3, 0), # t (x, y, z)
                         tf.transformations.quaternion_from_euler(0, 0, 0), # R (roll, pitch, yaw)
                         rospy.Time.now(), # time
                         "camera1_link", # child frame
                         "camera2_link") # parent frame

        rate.sleep()
