#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Created on 2015/02/11

@author: spiralray
'''

import shuttle

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import random

if __name__ == '__main__':
    # グラフ作成
    fig = plt.figure()
    ax = Axes3D(fig)
    
    # 軸ラベルの設定
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    
    # 表示範囲の設定
    ax.set_xlim(-10, 10)
    ax.set_ylim(0, 10)
    ax.set_zlim(0, 10)
    
    dt = 1.0/30.0
    
    
    tx = []
    ty = []
    tz = []
    truth = shuttle.Shuttle( np.mat([[0],[0],[0.79],[0],[20.6246594234],[26.3983602458],[0],[0],[0]]) )
    print "[{0:8.3f} {1:8.3f} {2:8.3f} {3:8.3f} {4:8.3f} {5:8.3f} {6:8.3f} {7:8.3f} {8:8.3f}]".format(truth.mu[0,0],truth.mu[1,0],truth.mu[2,0],truth.mu[3,0],truth.mu[4,0],truth.mu[5,0],truth.mu[6,0],truth.mu[7,0],truth.mu[8,0])
    
    x = []
    y = []
    z = []
    s = shuttle.Shuttle( np.mat([[0],[0],[0.79],[0],[20.6246594234],[26.3983602458],[0],[0],[0]]) )
    
    ox = []
    oy = []
    oz = []
    
    x.append(s.mu[0,0])
    y.append(s.mu[1,0])
    z.append(s.mu[2,0])
    
    old_x = s.mu[0,0]+random.uniform(-0.3,0.3)
    old_y = s.mu[1,0]+random.uniform(-0.3,0.3)
    old_z = s.mu[2,0]+random.uniform(-0.3,0.3)
    
    random.seed()
    
    for i in range(0, 500):
        for var in range(0, 10):
            truth.predict(dt/10)
        tx.append(truth.mu[0,0])
        ty.append(truth.mu[1,0])
        tz.append(truth.mu[2,0])
        
        #print "[{0:8.3f} {1:8.3f} {2:8.3f} {3:8.3f} {4:8.3f} {5:8.3f} {6:8.3f} {7:8.3f} {8:8.3f}]".format(truth.mu[0,0],truth.mu[1,0],truth.mu[2,0],truth.mu[3,0],truth.mu[4,0],truth.mu[5,0],truth.mu[6,0],truth.mu[7,0],truth.mu[8,0])
        
        for var in range(0, 10):
            s.predict(dt/10)
        
        obs_x = truth.mu[0,0]+random.uniform(-0.1,0.1)
        obs_y = truth.mu[1,0]+random.uniform(-0.1,0.1)
        obs_z = truth.mu[2,0]+random.uniform(-0.1,0.1)
        
        ox.append(obs_x)
        oy.append(obs_y)
        oz.append(obs_z)
        
        x.append(s.mu[0,0])
        y.append(s.mu[1,0])
        z.append(s.mu[2,0])
        
        obserbation_matrix = np.mat([[ obs_x  ],[ obs_y ],[ obs_z ],[ (obs_x-old_x)/dt ],[ (obs_y-old_y)/dt ],[ (obs_z-old_z)/dt ] ])
        s.update(obserbation_matrix)
        
        old_x = obs_x
        old_y = obs_y
        old_z = obs_z
        
        
        
    
    ax.scatter3D(tx, ty, tz, color=(1.0, 0.0, 0.0), marker="o", s=10)
    ax.scatter3D(ox, oy, oz, color=(0.0, 1.0, 0.0), marker="o", s=10)
    ax.scatter3D(x, y, z, color=(0.0, 0.0, 1.0), marker="o", s=10)
    
    plt.show()
    
        