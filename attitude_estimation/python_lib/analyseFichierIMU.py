# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 22:07:32 2015

Data analysis of IMU file

@author: S. Bertrand
"""

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
    
    plt.close('all')    
    imuMes.plotAcc()
    imuMes.plotGyr()
    imuMes.plotMag()
    imuMes.plotEulerAngles()
    plt.show()
    
    

    print( "var: \t mean (stdDev)")
    print( "---------------------------"    )
    print( "accX: \t" +str(np.mean(imuMes.accX)) + " (" + str(np.std(imuMes.accX)) +")")
    print( "accY: \t" +str(np.mean(imuMes.accY)) + " (" + str(np.std(imuMes.accY)) +")")
    print( "accZ: \t" +str(np.mean(imuMes.accZ)) + " (" + str(np.std(imuMes.accZ)) +")")
    print( "gyrX: \t" +str(np.mean(imuMes.gyrX)) + " (" + str(np.std(imuMes.gyrX)) +")")
    print( "gyrY: \t" +str(np.mean(imuMes.gyrY)) + " (" + str(np.std(imuMes.gyrY)) +")")
    print( "gyrZ: \t" +str(np.mean(imuMes.gyrZ)) + " (" + str(np.std(imuMes.gyrZ)) +")")
    print( "magX: \t" +str(np.mean(imuMes.magX)) + " (" + str(np.std(imuMes.magX)) +")")
    print( "magY: \t" +str(np.mean(imuMes.magY)) + " (" + str(np.std(imuMes.magY)) +")")
    print( "magZ: \t" +str(np.mean(imuMes.magZ)) + " (" + str(np.std(imuMes.magZ)) +")")
    print( "---------------------------")
    
    