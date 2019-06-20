# -*- coding: utf-8 -*-
"""
Created on Mon Nov 07 21:15:50 2016

@author: Hoagie
"""

import numpy as np
import KalmanFilter as kf
import matplotlib.pyplot as plt

nbPoints = 1000

# trajectoire vraie
acc = np.zeros(nbPoints)
acc[0:nbPoints/5] = np.linspace(0.0, 1.0, nbPoints/5)
acc[nbPoints/5:nbPoints*4/5] = np.linspace(1.0, 1.0, nbPoints*3/5)
acc[4*nbPoints/5:nbPoints] = np.linspace(1.0, 0.0, nbPoints/5)

Te = 0.01


temps = np.linspace(0,nbPoints-1, nbPoints) * Te



position = np.zeros(nbPoints)
velocity = np.zeros(nbPoints)


for i in range(0,nbPoints-1):
    velocity[i+1] = velocity[i] + Te * acc[i]
    position[i+1] = position[i] + Te * velocity[i] + Te**2/2 *acc[i] 
    

# mesures
np.random.seed(1)

#accélération bruitée
stdDevAcc = 2.50
meanAcc = 0.0
noiseAcc = np.random.normal(meanAcc, stdDevAcc, nbPoints)
accMeas = acc + noiseAcc


# position bruitée
stdDevPos = 0.50
meanPos = 0.0
noisePos = np.random.normal(meanPos, stdDevPos, nbPoints)
positionMeas = position + noisePos



nOverM = 1

# supression d'échantillons de mesures en position

if (nOverM!=1):
    positionMeas[0] = np.NaN

for i in range(1, nbPoints-1):
    if (np.mod(i,nOverM)!=0):
        positionMeas[i] = np.NaN
        
        
# x_dot = v
# xkp1 = xk + Te vk
        
nx = 2
nu = 1
ny = 1       
        
Ak = np.array( [[1.0, Te], [0.0, 1.0]] )
Bk = np.array( [[Te**2/2], [Te]] )
Ck = np.array( [[1.0, 0.0]] )       

Qk = np.array( [[ stdDevAcc**2 ]] )
Rk = np.array([[ stdDevPos**2 ]])

x0hat = np.array([[10.5] , [0.5]])
P0 = np.array( [[0.5**2 , 0.0],[0.0, 0.5**2]] ) 


positionKF = kf.KalmanFilter(nx, nu, ny)
positionKF.setStateEquation(Ak, Bk)
positionKF.setCk(Ck)
positionKF.setQk(Qk)
positionKF.setRk(Rk)
positionKF.initFilter( x0hat , P0 ) 



filteredState = np.zeros((2,nbPoints))
filteredState[:,0] = x0hat[:,0]

filteredStdDev = np.zeros_like(filteredState)
filteredStdDev[0,0] = np.sqrt(P0[0,0])
filteredStdDev[1,1] = np.sqrt(P0[1,1])


integratedStateFromAccMeas = np.zeros((2,nbPoints))
integratedStateFromAccMeas[:,0] = x0hat[:,0]

uk = np.zeros(1)
yk = np.zeros(1)


for i in range(1, nbPoints):

    integratedStateFromAccMeas[1,i] = integratedStateFromAccMeas[1,i-1] + Te*accMeas[i-1]
    integratedStateFromAccMeas[0,i] = integratedStateFromAccMeas[0,i-1] + Te*integratedStateFromAccMeas[1,i-1] +Te**2*accMeas[i-1]
    

    uk[0] = accMeas[i-1]   
    positionKF.predict(uk)
    
    if not(np.isnan(positionMeas[i])):
        #print 'update'
        yk[0] = positionMeas[i]
        positionKF.update(yk)
        
    filteredState[:,i] = positionKF.xk[:,0]
#    filteredStdDev[i] = np.sqrt(positionKF.Pk[0])


plt.close('all')

plt.figure()
plt.plot(temps, filteredState[0,:], 'b-')
plt.plot(temps, positionMeas, 'kx')
plt.plot(temps, position, 'r-')
plt.plot(temps, integratedStateFromAccMeas[0,:], 'g-')
plt.grid()


plt.figure()
plt.plot(temps, accMeas, 'kx')
plt.plot(temps, acc, 'r-')
plt.grid()



plt.figure()
plt.plot(temps,filteredState[0,:] - position, 'b-')
plt.plot(temps, integratedStateFromAccMeas[0,:] - position, 'g-')
#plt.plot(temps, 3*filteredStdDev, 'k-')
#plt.plot(temps, -3*filteredStdDev, 'k-')
plt.grid()

plt.show()