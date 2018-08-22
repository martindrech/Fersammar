#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 22 15:36:04 2018

@author: martindrech
"""

import sounddevice as sd
import numpy as np
import pylab as plt

f = 440
D = 2
fs = 44000


m = D*f
n = int(fs/f)
N = n*m



x = np.linspace(0, 2*np.pi*m, N)


data = np.sin(2*np.pi*f*x)
sd.default.samplerate = fs
#%%
sd.play(data)

#%%
plt.plot(x, data)