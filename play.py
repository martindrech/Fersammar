import sounddevice as sd
import numpy as np
import pylab as plt
import time

def playTone(f, D):
    #f = 220
    #D = 5
    fs = 44000
    
    
    m = D*f
    n = int(fs/f)
    N = n*m
    
    
    
    x = np.linspace(0, D, N)
    
    
    data = np.sin(2*np.pi*f*x)+np.sin(2*np.pi*f*1.5*x)+np.sin(2*np.pi*f*x*2)
    sd.default.samplerate = fs
    #time.sleep(D)
    sd.play(data)
    
#%%
tones = np.array([165, 175, 196, 220, 247, 262, 294, 330]*2)*2
for i in tones:
    playTone(i, .2)
    print(i)
#%%
playTone(262, 2)