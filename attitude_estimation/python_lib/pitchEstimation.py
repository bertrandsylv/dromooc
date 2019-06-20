ré# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 14:52:12 2015

Pitch estimation from gyro and accelero measurements
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
    gyroPitchRateStdDev = 0.0065*0.0065  # (rad/s)
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
    


    # init data structures for simulation
    estimatedPitch = []
    estimatedPitch.append(x0[0,0])
    estimatedBias = []
    estimatedBias.append(x0[1,0])
    indicesK = [0]
    
    pitchImu = []
    
    
    computedPitchFromAcc = []    
    computedPitchFromGyro = [0.0]

    for i in range(0, len(imuMes.time)):
        # read pitch rate measurement from gyro         
        uk = imuMes.gyrY[i]
        pitchKF.predict(uk)
    
        if (i==0):
            computedPitchFromGyro.append( computedPitchFromGyro + Te*uk )
        else:
            computedPitchFromGyro.append( computedPitchFromGyro[len(computedPitchFromGyro)-1] + Te*uk)
        
        
        # compute pitch from acc measurements
        accX  = imuMes.accX[i]
        accY = imuMes.accY[i]
        accZ = imuMes.accZ[i]                
        yk = math.atan2(-accX , math.sqrt( math.pow(accY,2) + math.pow(accZ,2) ) )

        computedPitchFromAcc.append(yk)    
    
        pitchKF.update(yk)

        
        
        estimatedPitch.append( pitchKF.xk[0,0])
        estimatedBias.append(pitchKF.xk[1,0])
        indicesK.append(i+1)       
        pitchImu.append( math.radians(  imuMes.pitchDeg[i] ) )
        
    
    
    
    fig1, graphArray = plt.subplots(3)
    
    graphArray[0].plot(indicesK, computedPitchFromGyro, color='k')
    graphArray[0].plot(indicesK[1:len(indicesK)], pitchImu, color='b')
    graphArray[0].set_title('Pitch estimation (blue: ref pitch from IMU filter)')
    graphArray[0].set_ylabel('From gyro (rad)')
    graphArray[0].grid(True)
        
    graphArray[1].plot(indicesK[1:len(indicesK)], computedPitchFromAcc, color='g')
    graphArray[1].plot(indicesK[1:len(indicesK)], pitchImu, color='b')
    graphArray[1].set_ylabel('From acc (rad)')    
    graphArray[1].grid(True)

    graphArray[2].plot(indicesK, estimatedPitch, color='r')
    graphArray[2].plot(indicesK[1:len(indicesK)], pitchImu, color='b')
    graphArray[2].set_ylabel('From KF (rad)')
    graphArray[2].set_xlabel('iteration')
    graphArray[2].grid(True)    
    
    fig2, graph = plt.subplots(1)
    graph.plot(indicesK, estimatedBias, color = 'r')
    graph.grid(True)
    graph.set_ylabel('gyro bias estimate (rad/s)')
    graph.set_xlabel('iteration')