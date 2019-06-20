# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 14:52:12 2015

Roll Pitch Yaw estimation from IMU measurements
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

    # system definition  (roll or pitch)
    # state : angle (rad), gyro's bias (rad/s)
    # input: angular speed (rad/s) from gyro measurement
    # measurement : angle (rad) computed from accelero measurements
    nx = 2
    nu = 1
    ny = 1    
    Ak = np.array( [[1.0, -Te],[0.0,1.0]] )
    Bk = np.array( [ [Te],[0.0] ] )
    Ck = np.array( [[1.0, 0.0]] )
    
    # Definition of KF for Roll estimation
    # -------------------------------------------
    # noises std dev
    gyroRollRateStdDev = 0.0070*0.0070  # (rad/s)
    gyroRollRateBiasStdDev = 0.00005  # (rad/s)
    acceleroRollStdDev = 0.01  # (rad)
    # cov matrices
    QkRoll = Te * np.array( [ [math.pow(gyroRollRateStdDev,2), 0.0], [0.0, math.pow(gyroRollRateBiasStdDev,2)] ] )
    RkRoll = np.array([math.pow(acceleroRollStdDev,2)])

    # initial state and covariance
    x0Roll = np.array([ [0.0],[-0.007] ])
    P0Roll = np.array( [ [0.5,0.0],[0.0,0.1] ] )  
    
    # filter instantiation
    KFRoll = kf.KalmanFilter(nx, nu, ny)
    KFRoll.setStateEquation(Ak, Bk)
    KFRoll.setCk(Ck)
    KFRoll.setQk(QkRoll)
    KFRoll.setRk(RkRoll)
    KFRoll.initFilter( x0Roll , P0Roll )
    

    # Definition of KF for Pitch estimation
    # -------------------------------------------
    # noises std dev
    gyroPitchRateStdDev = 0.0065*0.0065  # (rad/s)
    gyroPitchRateBiasStdDev = 0.00005  # (rad/s)
    acceleroPitchStdDev = 0.01  # (rad)
    # cov matrices
    QkPitch = Te * np.array( [ [math.pow(gyroPitchRateStdDev,2), 0.0], [0.0, math.pow(gyroPitchRateBiasStdDev,2)] ] )
    RkPitch = np.array([math.pow(acceleroPitchStdDev,2)])

    # initial state and covariance
    x0Pitch = np.array([ [0.0],[-0.006] ])
    P0Pitch = np.array( [ [0.5,0.0],[0.0,0.1] ] )  

    # filter instantiation
    KFPitch = kf.KalmanFilter(nx, nu, ny)
    KFPitch.setStateEquation(Ak, Bk)
    KFPitch.setCk(Ck)
    KFPitch.setQk(QkPitch)
    KFPitch.setRk(RkPitch)
    KFPitch.initFilter( x0Pitch , P0Pitch )
    



    # init data structures for simulation
    # ------------------------------------
    # Roll
    estimatedRoll = []
    estimatedRoll.append(x0Roll[0,0])
    estimatedBiasRoll = []
    estimatedBiasRoll.append(x0Roll[1,0])
    
    rollImu = []
    
    computedRollFromAcc = []    
    computedRollFromGyro = [0.0]

    # Pitch
    estimatedPitch = []
    estimatedPitch.append(x0Pitch[0,0])
    estimatedBiasPitch = []
    estimatedBiasPitch.append(x0Pitch[1,0])
    
    pitchImu = []
    
    computedPitchFromAcc = []    
    computedPitchFromGyro = [0.0]

    
    # yaw
    yawImu = []
    
    computedYawFromMag = []
    computedYawFromMag.append(0.)
    
    
    indicesK = [0]
    
    
    
    
    for i in range(0, len(imuMes.time)):
        # read angular speed measurements from gyro         
        ukRoll = imuMes.gyrX[i]
        ukPitch = imuMes.gyrY[i]
        
        # prediction step
        KFRoll.predict(ukRoll)
        KFPitch.predict(ukPitch)
    
    
        if (i==0):
            computedRollFromGyro.append( computedRollFromGyro + Te*ukRoll )
            computedPitchFromGyro.append( computedPitchFromGyro + Te*ukPitch )
        else:
            computedRollFromGyro.append( computedRollFromGyro[len(computedRollFromGyro)-1] + Te*ukRoll)
            computedPitchFromGyro.append( computedPitchFromGyro[len(computedPitchFromGyro)-1] + Te*ukPitch)
        
        # compute roll and pitch from acc measurements
        accX  = imuMes.accX[i]
        accY = imuMes.accY[i]
        accZ = imuMes.accZ[i]                
        
        ykRoll = math.atan2(accY , accZ)
        computedRollFromAcc.append(ykRoll)    
        
        ykPitch = math.atan2(-accX , math.sqrt( math.pow(accY,2) + math.pow(accZ,2) ) )

        computedPitchFromAcc.append(ykPitch)    
    


        # update step        
        KFRoll.update(ykRoll)
        KFPitch.update(ykPitch)

        
        
        
        # compute yaw
        phi =  KFRoll.xk[0,0]
        theta = KFPitch.xk[0,0]
        cp = np.cos(phi)
        sp = np.sin(phi)
        ct = np.cos(theta)
        st = np.sin(theta)
        
        Mat = np.array([[  ct , sp*st  ,  cp*st ],
                        [  0. , cp  ,  -sp ],
                        [ -st  , sp*ct  , cp*ct  ]])
        
        magX = imuMes.magX[i]
        magY = imuMes.magY[i]
        magZ = imuMes.magZ[i]
        
        magVecBdy = np.array([[magX] , [magY] , [magZ]])
        
        magVecInert = np.dot(Mat, magVecBdy)
            
        psi = math.atan2(-magVecInert[1], magVecInert[0])
        
        if (i>1):
            computedYawFromMag.append(psi-computedYawFromMag[1])
        else:
            computedYawFromMag.append(psi)    
    
    
        
        # save date for simulation
        estimatedRoll.append( KFRoll.xk[0,0])
        estimatedBiasRoll.append(KFRoll.xk[1,0])
        estimatedPitch.append( KFPitch.xk[0,0])
        estimatedBiasPitch.append(KFPitch.xk[1,0])
        
        
        indicesK.append(i+1)       
        
        
        rollImu.append( math.radians(  imuMes.rollDeg[i] ) )
        pitchImu.append( math.radians(  imuMes.pitchDeg[i] ) )
        yawImu.append( math.radians(  imuMes.yawDeg[i] ) )
    
    
    
    
    
    # angles
    fig1, graphArray = plt.subplots(3,2)
    
    # roll
    graphArray[0,0].plot(indicesK, computedRollFromGyro, color='k')
    graphArray[0,0].plot(indicesK[1:len(indicesK)], rollImu, color='b')
    graphArray[0,0].set_title('Roll estimation (blue: ref roll from IMU filter)')
    graphArray[0,0].set_ylabel('From gyro (rad)')
    graphArray[0,0].grid(True)
        
    graphArray[1,0].plot(indicesK[1:len(indicesK)], computedRollFromAcc, color='g')
    graphArray[1,0].plot(indicesK[1:len(indicesK)], rollImu, color='b')
    graphArray[1,0].set_ylabel('From acc (rad)')    
    graphArray[1,0].grid(True)

    graphArray[2,0].plot(indicesK, estimatedRoll, color='r')
    graphArray[2,0].plot(indicesK[1:len(indicesK)], rollImu, color='b')
    graphArray[2,0].set_ylabel('From KF (rad)')
    graphArray[2,0].set_xlabel('iteration')
    graphArray[2,0].grid(True)    
    
    # pitch
    graphArray[0,1].plot(indicesK, computedPitchFromGyro, color='k')
    graphArray[0,1].plot(indicesK[1:len(indicesK)], pitchImu, color='b')
    graphArray[0,1].set_title('Pitch estimation (blue: ref roll from IMU filter)')
    #graphArray[0,1].set_ylabel('From gyro (rad)')
    graphArray[0,1].grid(True)
        
    graphArray[1,1].plot(indicesK[1:len(indicesK)], computedPitchFromAcc, color='g')
    graphArray[1,1].plot(indicesK[1:len(indicesK)], pitchImu, color='b')
    #graphArray[1,1].set_ylabel('From acc (rad)')    
    graphArray[1,1].grid(True)

    graphArray[2,1].plot(indicesK, estimatedPitch, color='r')
    graphArray[2,1].plot(indicesK[1:len(indicesK)], pitchImu, color='b')
    #graphArray[2,1].set_ylabel('From KF (rad)')
    graphArray[2,1].set_xlabel('iteration')
    graphArray[2,1].grid(True)     
    
    
    # bias
    fig2, graph = plt.subplots(1,2)
    
    graph[0].plot(indicesK, estimatedBiasRoll, color = 'r')
    graph[0].grid(True)
    graph[0].set_ylabel('gyro bias estimate (rad/s)')
    graph[0].set_xlabel('iteration')
    graph[0].set_title('Roll')
    
    graph[1].plot(indicesK, estimatedBiasPitch, color = 'r')
    graph[1].grid(True)
    #graph[1].set_ylabel('Pitch : gyro bias estimate (rad/s)')
    graph[1].set_xlabel('iteration')
    graph[1].set_title('Pitch')
    
    
    
    # yaw
    fig1 = plt.figure()
    
    # roll
    plt.plot(indicesK, computedYawFromMag, color='k')
    plt.plot(indicesK[1:len(indicesK)], yawImu, color='b')
    plt.title('Yaw estimation (blue: ref yaw from IMU filter)')
    plt.ylabel('From magneto (rad)')
    plt.grid(True)