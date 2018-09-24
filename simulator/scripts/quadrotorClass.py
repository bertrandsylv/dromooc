#!/usr/bin/env python
"""
#DroMOOC (www.onera.fr/dromooc)
#ONERA-ENSTA-CentraleSupelec-Paris Saclay
#all variables are in SI unit
"""
import numpy as np


class Quadrotor:
    
    def __init__(self):
        
        # position and velocity (in intertial frame)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.Vx = 0.0
        self.Vy = 0.0
        self.Vz = 0.0
        
        #attitude angles
        self.phi = 0.0      # roll
        self.theta = 0.0    # pitch
        self.psi = 0.0      # yaw
                
        # angular velocity  (in body frame)
        self.Omega_p = 0.0
        self.Omega_q = 0.0
        self.Omega_r = 0.0
        
        # motors' speeds
        self.omega1 = 0.0
        self.omega2 = 0.0
        self.omega3 = 0.0
        self.omega4 = 0.0
        
        # parameters
        ''' completer avec bonnes valeurs '''
        self.g0 = 9.81
        self.m = 2.5        # mass
        self.J = np.eye(3)  # inertia matrix
        self.l = .30        # arm length
        self.b = 0.5        # aerodynamic coefficient
        self.d = 0.5        # aerodynamic coefficient

    


    def stateUpdateFromInput(self, inputVector, inputType = 'MotorSpeeds', Ts=0.001):
    # inputType : 'motorSpeed', 'thrustAndTorques'
        
        if (inputType == 'MotorSpeeds'):
            # motor speeds            
            self.omega1 = inputVector[0]
            self.omega2 = inputVector[1]
            self.omega3 = inputVector[2]
            self.omega4 = inputVector[3]
            
            # thrust
            T = self.b * (self.omega1**2+ self.omega2**2 + self.omega3**2 + self.omega4**2)
    
            # torque (in body frame)        
            Gamma1 = self.l * self.b * (-self.omega2**2 + self.omega4**2)
            Gamma2 = self.l * self.b * (self.omega1**2 - self.omega3**2)
            Gamma3 = self.d * (self.omega1**2 - self.omega2**2 + self.omega3**2 - self.omega4**2)
        
        elif (inputType == 'ThrustAndTorques'):   
            # thrust
            T = inputVector[0]

            # torque (in body frame)
            Gamma1 = inputVector[1]
            Gamma2 = inputVector[2]
            Gamma3 = inputVector[3]

            # motor speeds
            '''    A COMPLETER
            self.omega1 = 
            self.omega2 = 
            self.omega3 = 
            self.omega4 = 
            '''

        Gamma = np.array([[Gamma1], [Gamma2], [Gamma3]])

        # position and velocity
        Pos = np.array([[self.x],[self.y],[self.z]])
        Vit = np.array([[self.Vx],[self.Vy],[self.Vz]])
            
        # trigonometry
        cf = np.cos(self.phi)
        sf = np.sin(self.phi)
        ct = np.cos(self.theta)
        st = np.sin(self.theta)
        cp = np.cos(self.psi)
        sp = np.sin(self.psi)
    
        # orientation matrix
        R = np.array( [ [ct*cp, sf*st*cp-cf*sp, cf*st*cp+sf*sp] ,
                        [ct*sp, sf*st*sp+cf*cp, cf*st*sp-sf*cp] ,
                        [-st, sf*ct, cf*ct] ])   
        
        # angular velocity vector
        Omega = np.array([[self.Omega_p],[self.Omega_q],[self.Omega_r]])

        # skew symmetric matrix associated to vector cross product with Omega                    
        OmegaX = np.array([ [0, -self.Omega_r, self.Omega_q],
                            [self.Omega_r, 0, -self.Omega_p],
                            [-self.Omega_q, self.Omega_p, 0] ])
                            
        # linear acceleration
        Acc = np.array([[0.],[0.],[self.g0]]) - (T/self.m)*np.dot(R,np.array([[0.],[0.],[1.]])) 
        # !!!!! z vers le bas !!!!!        
        # PREVOIR FORCE DE PERTURBATION        
        
        
        # translationnal dynamics
        Pos += Ts*Vit
        Vit += Ts*Acc
        
        # orientation dynamics
        R += Ts*np.dot(R, OmegaX)
        Omega += Ts*(-np.cross(Omega.T, np.dot(self.J, Omega).T).T + Gamma )       
        
        
        # data update
        self.x = Pos[0][0]
        self.y = Pos[1][0]
        self.z = Pos[2][0]
        self.Vx = Vit[0][0]
        self.Vy = Vit[1][0]
        self.Vz = Vit[2][0]
        
        self.theta = - np.arcsin(R[2,0])
        self.psi = np.arctan(R[1,0]/R[0,0])
        self.phi = np.arctan(R[2,1]/R[2,2]);
        
        self.Omega_p = Omega[0][0]
        self.Omega_q = Omega[1][0]
        self.Omega_r = Omega[2][0]



if __name__=='__main__':
    quadrotor = Quadrotor()
    print quadrotor.x, quadrotor.y, quadrotor.z
    for t in range(0,1000):
        quadrotor.stateUpdateFromInput([0.01, 0.2, 0.2, 0.01])    
    print quadrotor.x, quadrotor.y, quadrotor.z