#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  4 19:11:16 2018

@author: siyuan
"""
import numpy as np
from scipy.fftpack import dst
from scipy.fftpack import idst
#import matplotlib.pyplot as plt

def fast_poisson(gx,gy):
    
#    j = 1:ydim-1; 
#	k = 1:xdim-1;
#
#	% Laplacian
#	gyy(j+1,k) = gy(j+1,k) - gy(j,k); 
#	gxx(j,k+1) = gx(j,k+1) - gx(j,k);
    
    
    m,n = gx.shape
    gxx = np.zeros((m,n))
    gyy = np.zeros((m,n))
    f = np.zeros((m,n))
    img = np.zeros((m,n))
    gyy[1:,:-1] = gy[1:,:-1] - gy[:-1,:-1]
    gxx[:-1,1:] = gx[:-1,1:] - gx[:-1,:-1]
    f = gxx + gyy 
    
    f2 = f[1:-1,1:-1].copy()
    
    f_sinx = dst(f2,norm='ortho')
    f_sinxy = dst(f_sinx.T,norm='ortho').T
    

    
    x_mesh, y_mesh = np.meshgrid(range(n-2),range(m-2)) 
    x_mesh = x_mesh +1
    y_mesh = y_mesh +1
    denom = (2*np.cos(np.pi*x_mesh/(n-1))-2) + (2*np.cos(np.pi*y_mesh/(m-1))-2)
    
    
    f3 = f_sinxy/denom
#    plt.figure(10)
#    plt.imshow(denom)
#    plt.show()
    f_realx = idst(f3,norm='ortho')
    f_realxy = idst(f_realx.T,norm='ortho').T
    img[1:-1,1:-1] = f_realxy.copy()
    return img
    
#%%