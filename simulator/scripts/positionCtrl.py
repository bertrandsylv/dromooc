#!/usr/bin/env python
"""
#DroMOOC (www.onera.fr/dromooc)
#ONERA-ENSTA-CentraleSupelec-Paris Saclay
#all variables are in SI unit
"""

import rospy
from nav_msgs.msg import Odometry
from simulator.msg import Thrust, RPYAngles, WayPoint, Angle
import numpy as np
import quadrotorClass
from dynamic_reconfigure.server import Server
from simulator.cfg import positionCtrlCFGConfig

# node init
rospy.init_node('positionCtrl', anonymous=False)


# publishers
# -----------
# Thrust
pubThrust = rospy.Publisher('thrust', Thrust, queue_size=50)
# Attitude angles as reference for attitude control
pubRPYAnglesRef = rospy.Publisher('RPYAnglesRef', RPYAngles, queue_size=50)


# control frequency
freq = 20.
Ts = 1./freq
rate = rospy.Rate(freq)

# quadrotor
quadrotor = quadrotorClass.Quadrotor()
# state components
pos = np.zeros((3,1))
vel = np.zeros((3,1))
# WP ref components
posRef = np.zeros((3,1))
velRef = np.zeros((3,1)) # zero for WP stabilization
# Yaw reference
yawRef = 0.


''' dynamic parameters '''
'''
# PID gains
kp = 0.5
ki = 0.
kd = 0.1
'''

# ----------------------------------------------------------------------------
def callbackDynParam(config, level):
# -----------------------------------------------------------------------------

    global kp, ki, kd

    kp = float("""{Kp}""".format(**config))
    ki = float("""{Ki}""".format(**config))
    kd = float("""{Kd}""".format(**config))

    return config
# -----------------------------------------------------------------------------


# server for dyamic parameters
srv = Server(positionCtrlCFGConfig, callbackDynParam)

# init dynamic parameters
kp = rospy.get_param('/positionCtrl/Kp', 1.3)
ki = rospy.get_param('/positionCtrl/Ki', 0.)
kd = rospy.get_param('/positionCtrl/Kd', 1.7)




# subscribers callbacks
# ----------------------

# -----------------------------------------------------------------------------
def callBackYawRef(data):
# -----------------------------------------------------------------------------
    global yawRef
    
    #read yaw reference from message
    yawRef = data.angle
# -----------------------------------------------------------------------------        


# -----------------------------------------------------------------------------
def callBackWP(data):
# -----------------------------------------------------------------------------
    global posRef
    
    #read position from WP message
    posRef[0] = data.position.x
    posRef[1] = data.position.y
    posRef[2] = data.position.z
# -----------------------------------------------------------------------------        



# -----------------------------------------------------------------------------
def callBackOdometry(data):
# -----------------------------------------------------------------------------
    global pos, vit
    
    # read position from odom msg    
    pos[0] = data.pose.pose.position.x
    pos[1] = data.pose.pose.position.y
    pos[2] = data.pose.pose.position.z
    # read linear velocity from odom msg    
    vel[0] = data.twist.twist.linear.x
    vel[1] = data.twist.twist.linear.y
    vel[2] = data.twist.twist.linear.z
# -----------------------------------------------------------------------------        
        
   
# subscribers
# ------------
rospy.Subscriber("odom", Odometry, callBackOdometry)
rospy.Subscriber("WayPoint", WayPoint, callBackWP)
rospy.Subscriber("YawRef", Angle, callBackYawRef)


# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()    
    #global quadrotor

    # init messages
    thrustMsg = Thrust()
    RPYAnglesRefMsg = RPYAngles()
    
    # variables
    acc = np.zeros((3,1))
    e3 = np.array([[0.],[0.],[1.]])
    
    # main loop
    while not rospy.is_shutdown():
        

        # PID for position control
        acc = -kp*(pos - posRef) -kd*(vel - velRef)

        # Thrust vector
        Tvec = quadrotor.m * (acc + quadrotor.g0*e3)
        
        # Thrust magnitude
        T = np.linalg.norm(Tvec , 2)

        # desired angles corresponding to Thrust vector direction, with given reference yaw 
        if (T==0.):
            rollRef = 0.
            pitchRef = 0.
        
        else: # T>0

            Rd_e3 = Tvec / T
            
            if ( np.abs(np.mod(yawRef,2.*np.pi)) == np.pi/2.):  # yaw = +/- pi/2
                pitchRef = np.sign(yawRef) * np.arctan(Rd_e3[1] / Rd_e3[2] )
                rollRef = np.sign(yawRef) * np.arctan( np.cos(pitchRef) *  Rd_e3[0] / Rd_e3[2] )

            else:
                pitchRef = np.arctan( np.cos(yawRef)*(Rd_e3[0]/Rd_e3[2]) + np.sin(yawRef)*(Rd_e3[1]/Rd_e3[2])  )            
                rollRef = np.arctan( np.sin(pitchRef)*np.tan(yawRef) - (Rd_e3[1]*np.cos(pitchRef))/(Rd_e3[2]*np.cos(yawRef)) )

    
        # msgs update        
        timeNow = rospy.Time.now()
        thrustMsg.header.seq += 1
        thrustMsg.header.stamp = timeNow     
        thrustMsg.thrust = T        
        
        RPYAnglesRefMsg.header.seq += 1
        RPYAnglesRefMsg.header.stamp = timeNow
        RPYAnglesRefMsg.roll = rollRef
        RPYAnglesRefMsg.pitch = pitchRef
        RPYAnglesRefMsg.yaw = yawRef
        
        
        # msgs publications
        pubThrust.publish(thrustMsg)
        pubRPYAnglesRef.publish(RPYAnglesRefMsg)
         


        rate.sleep()

# -----------------------------------------------------------------------------
