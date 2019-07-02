#!/usr/bin/env python
"""
#DroMOOC (www.onera.fr/dromooc)
#ONERA-ENSTA-CentraleSupelec-Paris Saclay
#all variables are in SI unit
"""

import rospy
from attitude_estimation.msg import RPYAngles
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu, MagneticField
import tf
import numpy as np
#from dynamic_reconfigure.server import Server
#from simulator.cfg import attitudeCtrlCFGConfig



# node init
rospy.init_node('pitchEstimation', anonymous=False)


# publishers
# -----------
# estimated RPY angles
pubEstimatedRPY = rospy.Publisher('EstimatedRPYAngles', RPYAngles, queue_size=50)
# ground truth RPY angles
pubGroundTruthRPY = rospy.Publisher('GRoundTruthRPYAngles', RPYAngles, queue_size=50)

# measurements
accMeas = np.zeros((3,1))
gyroMeas = np.zeros((3,1))
magMeas = np.zeros((3,1))

# estimated attitude and angular velocity
yawEstim = 0.
pitchEstim = 0.
rollEstim = 0.
angularVelEstim = np.zeros((3,1))

# ground truth
yawGroundTruth = 0.
pitchGroundTruth = 0.
rollGroundTruth = 0.
quaternionGroundTruth = Quaternion()


# subscribers callbacks
# ----------------------

# -----------------------------------------------------------------------------
def callBackImuAcceleroGyro(data):
# -----------------------------------------------------------------------------
    global accMeas, gyroMeas, quaternionGroundTruth, rollGroundTruth, pitchGroundTruth, yawGroundTruth
    
    #read acceleration measurements
    accMeas[0] = data.linear_acceleration.x
    accMeas[1] = data.linear_acceleration.y    
    accMeas[2] = data.linear_acceleration.z    
    
    #read angular velocity measurements
    gyroMeas[0] = data.angular_velocity.x
    gyroMeas[1] = data.angular_velocity.y    
    gyroMeas[2] = data.angular_velocity.z        

    # read quaternion for ground truth
    quaternionGroundTruth.x = data.orientation.x
    quaternionGroundTruth.y = data.orientation.y
    quaternionGroundTruth.z = data.orientation.z
    quaternionGroundTruth.w = data.orientation.w
    
    anglesGroundTruth = tf.transformations.euler_from_quaternion([quaternionGroundTruth.x, quaternionGroundTruth.y, quaternionGroundTruth.z, quaternionGroundTruth.w], axes='sxyz')
    rollGroundTruth = anglesGroundTruth[0]
    pitchGroundTruth = anglesGroundTruth[1]
    yawGroundTruth = anglesGroundTruth[2]
    
    print((rollGroundTruth, pitchGroundTruth, yawGroundTruth))
    
    
# -----------------------------------------------------------------------------        



# -----------------------------------------------------------------------------
def callBackImuMagneto(data):
# -----------------------------------------------------------------------------
    global magMeas
    
    magMeas[0] = data.magnetic_field.x
    magMeas[1] = data.magnetic_field.y
    magMeas[2] = data.magnetic_field.z
    #print(magMeas)
# -----------------------------------------------------------------------------        

          
# subscribers
# ------------
rospy.Subscriber("/imu/data", Imu, callBackImuAcceleroGyro)
rospy.Subscriber("/imu/mag", MagneticField, callBackImuMagneto)


# **************************************** REPRENDRE EN DESSOUS  *************



# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    rospy.spin()    
    #global quadrotor

#    # init messages
#    #torqueMsg = Torque()
#    
#    # main loop
#    while not rospy.is_shutdown():
#
#        # attitude and reference        
#        eta = np.array([[roll],[pitch],[yaw]])
#        etaRef = np.array([[rollRef],[pitchRef],[yawRef]])
#
#        # PID for attitude control
#        u = -kp*(eta - etaRef) -kd*(angularVel - angularVelRef)
#
#        OmegaxJOmega = np.cross(angularVel.T, np.dot(quadrotor.J, angularVel).T).T
#        
#        # yaw        
#        yawErr = yawRef - yaw
#        # TO DO: COMPLETE MODULOSASSESSMENT OF THE GROUND RISK IN THE OPERATION OF SMALL UNMANNED AERIAL VEHICLES 
#        if (yawErr>np.pi):
#            yawErr = yawErr - 2.*np.pi
#        elif (yawErr<= -np.pi):
#            yawErr = yawErr + 2.*np.pi
#        yawErr = -yawErr
#        
#        uz = -kpYaw*(yawErr) -kdYaw*(angularVel[2] - angularVelRef[2])
#        u[2] = uz          
#        
#        torque = OmegaxJOmega + np.dot(quadrotor.J, u)
#        
#        # msgs update        
#        timeNow = rospy.Time.now()
#        torqueMsg.header.seq += 1
#        torqueMsg.header.stamp = timeNow     
#        torqueMsg.torque.x = torque[0]        
#        torqueMsg.torque.y = torque[1]
#        torqueMsg.torque.z = torque[2]
#        
#        
#        # msgs publications
#        pubTorque.publish(torqueMsg)
#         
#
#        rate.sleep()

# -----------------------------------------------------------------------------
