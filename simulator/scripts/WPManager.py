#!/usr/bin/env python
"""
#DroMOOC (www.onera.fr/dromooc)
#ONERA-ENSTA-CentraleSupelec-Paris Saclay
#all variables are in SI unit
"""

import rospy
from nav_msgs.msg import Odometry
from simulator.msg import WayPoint
import numpy as np
import quadrotorClass


# node init
rospy.init_node('WPManager', anonymous=False)




# publishers
# -----------
# Thrust
pubWP = rospy.Publisher('/WayPoint', WayPoint, queue_size=50)

# frequency
freq = 20.
Ts = 1./freq
rate = rospy.Rate(freq)

# state components
pos = np.zeros((3,1))
# WP ref components
posRef = np.zeros((3,1))


WPList = [  np.array([[1.],[-1.],[1.]]) , np.array([[1.],[1.],[1.]]) , np.array([[-1.],[1.],[1.]]), np.array([[-1.],[-1.],[1.]])  ]


# -----------------------------------------------------------------------------
def callBackOdometry(data):
# -----------------------------------------------------------------------------
    global pos
    
    # read position from odom msg    
    pos[0] = data.pose.pose.position.x
    pos[1] = data.pose.pose.position.y
    pos[2] = data.pose.pose.position.z
# -----------------------------------------------------------------------------        
        
   
# subscribers
# ------------
rospy.Subscriber("odom", Odometry, callBackOdometry)



# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()    
    #global quadrotor

    # init messages
    WPMsg = WayPoint()
    
    posRef = WPList.pop(0)
    WPList.append(posRef)
    
    # main loop
    while not rospy.is_shutdown():
        

        # distance to current WP
        dist = np.sqrt(np.linalg.norm(pos - posRef ,2))

        # switch to next WP
        if (dist<=0.3):
            posRef = WPList.pop(0)
            WPList.append(posRef)
        
    
        # msgs update        
        timeNow = rospy.Time.now()
        WPMsg.header.seq += 1
        WPMsg.header.stamp = timeNow     
        WPMsg.position.x = posRef[0]
        WPMsg.position.y = posRef[1]
        WPMsg.position.z = posRef[2]        
        
        # msgs publications
        pubWP.publish(WPMsg)
        
        rate.sleep()

# -----------------------------------------------------------------------------
