# -*- coding: utf-8 -*-
"""
Created on Fri Oct 30 21:18:32 2015

@author: S. Bertrand
"""

import numpy as np
import matplotlib.pyplot as plt

class ImuData:
    
    def __init__(self, CSVFileName, delim, Te):
        
        dataCSV = np.genfromtxt(CSVFileName, delimiter=delim)

        self.counter = dataCSV[:,0]
        self.time = (dataCSV[:,0]-dataCSV[0,0])*Te
        self.accX = dataCSV[:,1]
        self.accY = dataCSV[:,2]
        self.accZ = dataCSV[:,3]
        self.gyrX = dataCSV[:,4]
        self.gyrY = dataCSV[:,5]
        self.gyrZ = dataCSV[:,6]
        self.magX = dataCSV[:,7]
        self.magY = dataCSV[:,8]
        self.magZ = dataCSV[:,9]
        self.rollDeg = dataCSV[:,10]
        self.pitchDeg = dataCSV[:,11]
        self.yawDeg = dataCSV[:,12]


    def plotAcc(self):
        figAcc, graphArray = plt.subplots(3)
        # accelero    
        graphArray[0].plot(self.time , self.accX , color='r')
        graphArray[0].grid(True)
        graphArray[0].set_title('acc (m/s2)')
        graphArray[0].set_ylabel('x')
        graphArray[1].plot(self.time , self.accY , color='g')    
        graphArray[1].grid(True)
        graphArray[1].set_ylabel('y')
        graphArray[2].plot(self.time , self.accZ , color='b')
        graphArray[2].grid(True)
        graphArray[2].set_ylabel('z')
        graphArray[2].set_xlabel('time (s)')


    def plotGyr(self):
        figGyr, graphArray = plt.subplots(3)  
        graphArray[0].plot(self.time , self.gyrX , color='r')
        graphArray[0].grid(True)
        graphArray[0].set_title('gyro (rad/s)')    
        graphArray[0].set_ylabel('x')
        graphArray[1].plot(self.time , self.gyrY , color='g')    
        graphArray[1].grid(True)
        graphArray[1].set_ylabel('y')
        graphArray[2].plot(self.time , self.gyrZ , color='b')
        graphArray[2].grid(True)
        graphArray[2].set_ylabel('z')
        graphArray[2].set_xlabel('time (s)')
        
        
    def plotMag(self):
        figMag, graphArray = plt.subplots(3)
        graphArray[0].plot(self.time , self.magX , color='r')
        graphArray[0].grid(True)
        graphArray[0].set_title('mag ()')    
        graphArray[0].set_ylabel('x')
        graphArray[1].plot(self.time , self.magY , color='g')    
        graphArray[1].grid(True)
        graphArray[1].set_ylabel('y')
        graphArray[2].plot(self.time , self.magZ , color='b')
        graphArray[2].grid(True)
        graphArray[2].set_ylabel('z')
        graphArray[2].set_xlabel('time (s)')

    def plotEulerAngles(self):
        figEuler, graphArrayf2 = plt.subplots(3)
        graphArrayf2[0].plot(self.time, self.rollDeg, color='r')
        graphArrayf2[0].grid(True)
        graphArrayf2[0].set_title('Euleur angles (deg)')
        graphArrayf2[0].set_ylabel('roll')
        
        graphArrayf2[1].plot(self.time, self.pitchDeg, color='g')
        graphArrayf2[1].grid(True)
        graphArrayf2[1].set_ylabel('pitch')
    
        graphArrayf2[2].plot(self.time, self.yawDeg, color='b')
        graphArrayf2[2].grid(True)
        graphArrayf2[2].set_ylabel('yaw')
        graphArrayf2[2].set_xlabel('time (s)')


if __name__=='__main__':
    
    imuMes = ImuData('donneesIMUAuRepos.txt', '\t', 0.01)
    
    plt.close('all')    
    
    imuMes.plotAcc()
    imuMes.plotGyr()
    imuMes.plotMag()
    imuMes.plotEulerAngles()
    
    plt.show()