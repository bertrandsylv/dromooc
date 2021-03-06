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
import math
import KalmanFilter as kf
#from dynamic_reconfigure.server import Server
#from simulator.cfg import attitudeCtrlCFGConfig



# node init
rospy.init_node('KFPitchEstimation', anonymous=False)


# publishers
# -----------
# estimated RPY angles
pubEstimatedRPY = rospy.Publisher('EstimatedRPYAngles', RPYAngles, queue_size=50)
# ground truth RPY angles
pubGroundTruthRPY = rospy.Publisher('GroundTruthRPYAngles', RPYAngles, queue_size=50)


# sampling period (s)
Te = 1.0/25.0   # IMU Frequency: 25Hz
IMURate = rospy.Rate(1./Te)

# system definition
# state : pitch angle (rad), gyro's bias (rad/s)
# input: pitch rate (rad/s) from gyro measurement
# measurement : pitch angle (rad) computed from accelero measurements
nx = 2
nu = 1
ny = 1    
Ak = np.array( [[1.0, -Te],[0.0,1.0]] )
Bk = np.array( [ [Te],[0.0] ] )
Ck = np.array( [[1.0, 0.0]] )
    
# noises std dev
gyroPitchRateStdDev = 0.0065 #0.0065*0.0065  # (rad/s)
gyroPitchRateBiasStdDev = 0.00005  # (rad/s)
acceleroPitchStdDev = 0.01  # (rad)
# cov matrices
Qk = Te * np.array( [ [math.pow(gyroPitchRateStdDev,2), 0.0], [0.0, math.pow(gyroPitchRateBiasStdDev,2)] ] )
Rk = np.array([math.pow(acceleroPitchStdDev,2)])

# initial state and covariance
x0 = np.array([ [0.0],[-0.006] ])
P0 = np.array( [ [0.5,0.0],[0.0,0.1] ] )  
    
# filter instantiation
pitchKF = kf.KalmanFilter(nx, nu, ny)
pitchKF.setStateEquation(Ak, Bk)
pitchKF.setCk(Ck)
pitchKF.setQk(Qk)
pitchKF.setRk(Rk)
pitchKF.initFilter( x0 , P0 )

 


# measurements
accMeas = np.zeros((3,1))
gyroMeas = np.zeros((3,1))
magMeas = np.zeros((3,1))

# estimated attitude and angular velocity
yawEstim = 0.
pitchEstim = 0.
rollEstim = 0.
msgEstimatedRPY = RPYAngles()


# ground truth
yawGroundTruth = 0.
pitchGroundTruth = 0.
rollGroundTruth = 0.
quaternionGroundTruth = Quaternion()
msgGroundTruthRPY = RPYAngles()


# subscribers callbacks
# ----------------------

# -----------------------------------------------------------------------------
def callBackImuAcceleroGyro(data):
# -----------------------------------------------------------------------------
    global accMeas, gyroMeas
    global quaternionGroundTruth, pitchGroundTruth, rollGroundTruth, yawGroundTruth
    global pitchEstim, pitchKF
    
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
    
    #print((rollGroundTruth, pitchGroundTruth, yawGroundTruth))
    
    # read pitch rate measurement from gyro         
    uk = gyroMeas[1,0]
    pitchKF.predict(uk)
    
    # compute pitch measurement from accelero    
    yk = math.atan2(-accMeas[0,0] , math.sqrt( math.pow(accMeas[1,0],2) + math.pow(accMeas[2,0],2) ) )
    pitchKF.update(yk)

    pitchEstim = pitchKF.xk[0,0]
    
    
      
    
# -----------------------------------------------------------------------------        



## -----------------------------------------------------------------------------
#def callBackImuMagneto(data):
## -----------------------------------------------------------------------------
#    global magMeas
#    
#    magMeas[0] = data.magnetic_field.x
#    magMeas[1] = data.magnetic_field.y
#    magMeas[2] = data.magnetic_field.z
#    #print(magMeas)
## -----------------------------------------------------------------------------        

          
# subscribers
# ------------
rospy.Subscriber("/imu/data", Imu, callBackImuAcceleroGyro)
#rospy.Subscriber("/imu/mag", MagneticField, callBackImuMagneto)




# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
#    rospy.spin()    

    while not rospy.is_shutdown():
        
        timeNow = rospy.Time.now()
    
        msgEstimatedRPY.header.seq = msgEstimatedRPY.header.seq + 1
        msgEstimatedRPY.header.stamp = timeNow
        msgEstimatedRPY.roll = np.nan # not a number (not computed)
        msgEstimatedRPY.pitch = pitchEstim
        msgEstimatedRPY.yaw =  np.nan # not computed (not computed)
        pubEstimatedRPY.publish(msgEstimatedRPY)
    
        msgGroundTruthRPY.header.seq = msgGroundTruthRPY.header.seq + 1
        msgGroundTruthRPY.header.stamp = timeNow
        msgGroundTruthRPY.roll = rollGroundTruth
        msgGroundTruthRPY.pitch =  pitchGroundTruth
        msgGroundTruthRPY.yaw = yawGroundTruth
        pubGroundTruthRPY.publish(msgGroundTruthRPY)  
        
        IMURate.sleep()
# -----------------------------------------------------------------------------
