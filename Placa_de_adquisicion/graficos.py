# -*- coding: utf-8 -*-
"""
Created on Tue Nov 20 22:39:20 2018

"""

import numpy as np
from matplotlib import pyplot as plt
import os
import glob
#%%
#Mediciones 31 de octubre
oct_31_sin_pert = np.load('F:/Facultad/Instumentacion/PID/20181031/PID_sin_perturbar.npy')
oct_31_pert = np.load('F:/Facultad/Instumentacion/PID/20181031/PID_perturbado_1.npy')

plt.figure()
plt.plot(oct_31_sin_pert[0],oct_31_sin_pert[1])
plt.xlabel('tiempo (s)')
plt.ylabel('voltaje (V)')
plt.title('Sin perturbar')
plt.grid()

plt.figure()
plt.plot(oct_31_pert[0],oct_31_pert[1])
plt.xlabel('tiempo (s)')
plt.ylabel('voltaje (V)')
plt.title('Perturbado')
plt.grid()

#%%
#Mediciones 7 de noviembre
#miramos todas a ver que hay
path_nov_7 = 'F:/Facultad/Instumentacion/PID/20181107'
files = sorted(glob.glob(os.path.join(path_nov_7, '*.npy')))
N_files = len(files)
        
plt.figure()
for file in files:
    t, m, c = np.load(file)
    plt.plot(t, m)
plt.grid()

#%%
#elegimos algunas, perturbadas a ciertas frecuencias
frecs = ['.1', '.2', '.3', '.4', '.6', '10']
plt.figure()
for frec in frecs:
    filename = '/pid_sin_ruido_0.2_0.01_0.01_4_perturbacion_%sHz.npy' %frec
    t, m, c = np.load(path_nov_7+filename)
    plt.plot(t, m, label = str(frec))
plt.grid()
plt.legend()

#%%
#Mediciones de 14 de noviembre
path_nov_14 = 'F:/Facultad/Instumentacion/PID/20181114/'
for buffer_size in range(1,10):
    filename = 'pid_sin_ruido_0.06_0.05_0.003_8_caja_abierta_conbuffer_%s.npy' %buffer_size
    t, m, c = np.load(path_nov_14+filename)
    plt.plot(t, m, label = str(buffer_size))
plt.grid()
plt.legend()
plt.xlabel('tiempo (s)')
plt.ylabel('voltaje (V)')

#%%

file = 'pid_0.06_0.05_0.003_8_caja_abierta_conruido_500_mHz_mastenue.npy'
t, m, c = np.load(path_nov_14+file)
plt.plot(t, m, '.--')
plt.grid()
#plt.legend()
plt.xlabel('tiempo (s)')
plt.ylabel('voltaje (V)')