# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 14:52:12 2015

Roll estimation from gyro and accelero measurements
usign Kalman FIlter (small angles assumption)

@author: S. Bertrand
"""


import math
import KalmanFilter as kf
import numpy as np
import matplotlib.pyplot as plt
import ImuData as imud



if __name__=='__main__':

    # sampling period (s)
    Te = 0.01
    
    # read data file
    #fileName = 'donneesIMUAuRepos.txt'
    fileName = 'donneesIMUPetitsAngles.txt'
    delimiter = '\t'
    imuMes = imud.ImuData(fileName, delimiter, Te)

    # system definition
    # state : roll angle (rad), gyro's bias (rad/s)
    # input: roll rate (rad/s) from gyro measurement
    # measurement : roll angle (rad) computed from accelero measurements
    nx = 2
    nu = 1
    ny = 1    
    Ak = np.array( [[1.0, -Te],[0.0,1.0]] )
    Bk = np.array( [ [Te],[0.0] ] )
    Ck = np.array( [[1.0, 0.0]] )
    
    # noises std dev
    gyroRollRateStdDev = 0.0070*0.0070  # (rad/s)
    gyroRollRateBiasStdDev = 0.00005  # (rad/s)
    acceleroRollStdDev = 0.01  # (rad)
    # cov matrices
    Qk = Te * np.array( [ [math.pow(gyroRollRateStdDev,2), 0.0], [0.0, math.pow(gyroRollRateBiasStdDev,2)] ] )
    Rk = np.array([math.pow(acceleroRollStdDev,2)])

    # initial state and covariance
    x0 = np.array([ [0.0],[-0.007] ])
    P0 = np.array( [ [0.5,0.0],[0.0,0.1] ] )  
    
    # filter instantiation
    rollKF = kf.KalmanFilter(nx, nu, ny)
    rollKF.setStateEquation(Ak, Bk)
    rollKF.setCk(Ck)
    rollKF.setQk(Qk)
    rollKF.setRk(Rk)
    rollKF.initFilter( x0 , P0 )
    


    # init data structures for simulation
    estimatedRoll = []
    estimatedRoll.append(x0[0,0])
    estimatedBias = []
    estimatedBias.append(x0[1,0])
    indicesK = [0]
    
    rollImu = []
    
    
    computedRollFromAcc = []    
    computedRollFromGyro = [0.0]

    for i in range(0, len(imuMes.time)):
        # read roll rate measurement from gyro         
        uk = imuMes.gyrX[i]
        rollKF.predict(uk)
    
        if (i==0):
            computedRollFromGyro.append( computedRollFromGyro + Te*uk )
        else:
            computedRollFromGyro.append( computedRollFromGyro[len(computedRollFromGyro)-1] + Te*uk)
        
        
        # compute roll from acc measurements
        accX  = imuMes.accX[i]
        accY = imuMes.accY[i]
        accZ = imuMes.accZ[i]                
        yk = math.atan2(accY , accZ)

        computedRollFromAcc.append(yk)    
    
        rollKF.update(yk)

        
        
        estimatedRoll.append( rollKF.xk[0,0])
        estimatedBias.append(rollKF.xk[1,0])
        indicesK.append(i+1)       
        rollImu.append( math.radians(  imuMes.rollDeg[i] ) )
        
    
    
    
    fig1, graphArray = plt.subplots(3)
    
    graphArray[0].plot(indicesK, computedRollFromGyro, color='k')
    graphArray[0].plot(indicesK[1:len(indicesK)], rollImu, color='b')
    graphArray[0].set_title('Roll estimation (blue: ref roll from IMU filter)')
    graphArray[0].set_ylabel('From gyro (rad)')
    graphArray[0].grid(True)
        
    graphArray[1].plot(indicesK[1:len(indicesK)], computedRollFromAcc, color='g')
    graphArray[1].plot(indicesK[1:len(indicesK)], rollImu, color='b')
    graphArray[1].set_ylabel('From acc (rad)')    
    graphArray[1].grid(True)

    graphArray[2].plot(indicesK, estimatedRoll, color='r')
    graphArray[2].plot(indicesK[1:len(indicesK)], rollImu, color='b')
    graphArray[2].set_ylabel('From KF (rad)')
    graphArray[2].set_xlabel('iteration')
    graphArray[2].grid(True)    
    
    fig2, graph = plt.subplots(1)
    graph.plot(indicesK, estimatedBias, color = 'r')
    graph.grid(True)
    graph.set_ylabel('gyro bias estimate (rad/s)')
    graph.set_xlabel('iteration')