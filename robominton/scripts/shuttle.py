#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 2015/02/11

@author: spiralray
'''


import numpy as np
import math

class Shuttle:
    resist_coeff = 0.0007
    gravity = 9.79708
    mass = 0.0052

    
    def __init__(self, mu):
        self.mu = mu
        
        self.PEst = np.eye(9)*10
        
        self.Q = np.diag([0.05,0.05,0.05,0.1,0.1,0.1,1.,1.,1.])*(10**-2)
        self.R = np.diag([0.05,0.05,0.05])*(10**-1.5)
        
        #To set acceleration
        self.predict(0.00)
        
        self.PEst = np.diag([1,1,1,5.,5.,5.,1.,1.,1.])
        
    def getB(self, period):
        return np.mat([
            [0],
            [0],
            [(period**2)/2],
            [0],
            [0],
            [period],
            [0],
            [0],
            [1] ])

    def getA(self, mu, period):   #Observation matrix
        airR = -self.resist_coeff * math.sqrt(mu[3,0]**2 + mu[4,0]**2 + mu[5,0]**2 ) / self.mass
        return np.mat([
            [1,0,0,period,0,0,(period**2)/2,0,0],
            [0,1,0,0,period,0,0,(period**2)/2,0],
            [0,0,1,0,0,period,0,0,(period**2)/2],
            [0,0,0,1,0,0,period,0,0],
            [0,0,0,0,1,0,0,period,0],
            [0,0,0,0,0,1,0,0,period],
            [0,0,0,airR,0,0,0,0,0],
            [0,0,0,0,airR,0,0,0,0],
            [0,0,0,0,0,airR,0,0,0]
        ])
    
    def Jh(self, mu):   #Observation matrix
        return np.mat([
            [1,0,0,0,0,0,0,0,0],
            [0,1,0,0,0,0,0,0,0],
            [0,0,1,0,0,0,0,0,0]
        ])
        
    def predict(self,period):
        
        self.A = self.getA(self.mu,period)
        self.B = self.getB(period)
        self.u = np.mat([[-self.gravity]])
        # prediction
        self.mu = self.A * self.mu + self.B * self.u
        self.PPred = self.Q + self.A * self.PEst * self.A.T
        
    def update(self, Y):    #Y Observation value
        H = self.Jh(self.mu)
        y = Y - H * self.mu
        S = H * self.PPred * H.T + self.R
        K = self.PPred * H.T * S.I
        self.mu = self.mu + K * y
        self.PEst = ( np.eye( math.sqrt(np.size(self.PEst)) ) - K * H) * self.PPred
        
    def getState(self):
        return self.mu

