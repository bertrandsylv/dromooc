#!/usr/bin/env python
"""
#DroMOOC (www.onera.fr/dromooc)
#ONERA-ENSTA-CentraleSupelec-Paris Saclay
#all variables are in SI unit
"""

import rospy
import tf
from geometry_msgs.msg import Quaternion, Wrench
from nav_msgs.msg import Odometry
import numpy as np


# node init
rospy.init_node('quadrotorCtrl', anonymous=False)


# publishers
# -----------
# odometry
pubWrench = rospy.Publisher('/wrench', Wrench, queue_size=50)



# control frequency
fctrl = 20.
Tctrl = 1./fctrl
ctrlRate = rospy.Rate(fctrl)


''' a passer en param dynamique '''
WP = np.array([[0.0], [0.0], [1.0]])

# state components
pos = np.zeros((3,1))
vel = np.zeros((3,1))
eta = np.zeros((3,1))  # phi, theta, psi
omega = np.zeros((3,1))

# subscribers callbacks
# ----------------------


# -----------------------------------------------------------------------------
def callBackOdometry(data):
# -----------------------------------------------------------------------------
    global pos, vit, eta, omega
    
    # read position from odom msg    
    pos[0] = data.pose.pose.position.x
    pos[1] = data.pose.pose.position.y
    pos[2] = data.pose.pose.position.z
    # read linear velocity from odom msg    
    vel[0] = data.twist.twist.linear.x
    vel[1] = data.twist.twist.linear.y
    vel[2] = data.twist.twist.linear.z
    # read attitude angles from odom msg    
    quat = Quaternion()
    quat.x = data.pose.pose.orientation.x
    quat.y = data.pose.pose.orientation.y
    quat.z = data.pose.pose.orientation.z
    quat.w = data.pose.pose.orientation.w
    angles = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w], axes='sxyz')
    eta[0] = angles[0] # roll angle phi
    eta[1] = angles[1] # pitch angle theta
    eta[2] = angles[2] # yaw angle psi
    # read angular speed from odom msg
    omega[0] = data.twist.twist.angular.x
    omega[1] = data.twist.twist.angular.y
    omega[2] = data.twist.twist.angular.z
    
    
            
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

    wrenchMsg = Wrench()
    t = rospy.get_time()
    
    while not rospy.is_shutdown():
        timeNow = rospy.Time.now()

        # PD for position control

        # desired angles and thrust

        # PD for attitude control

        
        # wrenchMsg.xxx =
         
        # msgs publications
        pubWrench.publish(wrenchMsg)
         


        ctrlRate.sleep()

# -----------------------------------------------------------------------------
