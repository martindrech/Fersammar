#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 20 12:44:09 2018

@author: martindrech
"""

import os
os.chdir('/home/martindrech/Nextcloud/Documents/instrumentacion/Fersammar/Placa_de_adquisicion')

import pylab as plt
import numpy as np
#%%
"""
Barrido en p
"""


ki, kd, setp = 0, 0, 8


fig, (ax1, ax2) = plt.subplots(1, 2, num = 'barrido en p', figsize = (10, 4))

kps = np.linspace(0, 0.2, 20)

standard_deviations = []
averages = []

n1, n2 = 0, 15

for kp in kps[n1:n2]:
    filename = './20181114/pid_sin_ruido_%s_%s_%s_%s_caja_abierta_barrido_p.npy' % (kp, ki, kd, setp)
    t, m, c = np.load(filename)
    std_dev = np.std(m[t>.2])
    standard_deviations.append(std_dev)
    averages.append(np.average(m[t>2]))
    ax1.plot(t, m, label = '%.2f' % (kp) )
    
    
    


ax1.legend(ncol = 2, title = 'Valores de P:')
ax1.set_xlabel('Tiempo [s]')
ax1.set_ylabel('Voltage en fotodiodo [V]')
ax1.grid()

ax2.plot(kps[n1:n2], standard_deviations, 'o-')
ax2.set_xlabel(r'Valores de P [V/V]')
ax2.set_ylabel('Desviación estandar [V]')
ax2.grid()

plt.tight_layout()
#plt.savefig('barrido_en_p', dpi = 180)

#%%
"""
Barrido en I
"""

kp, kd, setp = 0.06, 0, 8

fig, (ax1, ax2) = plt.subplots(1, 2, num = 'barrido en I', figsize = (10, 4))

kis = np.linspace(0, 0.5, 21)
n1, n2 = 0, 15
standard_deviations = []


for ki in kis[n1:n2]:
    filename = './20181114/pid_sin_ruido_%s_%s_%s_%s_caja_abierta_barrido_I_sinbuffer.npy' % (kp, ki, kd, setp)
    t, m, c = np.load(filename)
    std_dev = np.std(m[t>.2])
    standard_deviations.append(std_dev)
    ax1.plot(t, m, label = '%.2f' % (ki) )

ax1.legend(ncol = 2)
ax1.set_xlabel('Tiempo [s]')
ax1.set_ylabel('Voltage en fotodiodo [V]')
ax1.grid()

ax2.plot(kps[n1:n2], standard_deviations, 'o-')
ax2.set_xlabel(r'Valores de I [V/V]')
ax2.set_ylabel('Desviación estandar [V]')
ax2.grid()
plt.tight_layout()

plt.savefig('barrido_en_I', dpi = 180)
#%%
"""
Barrido en D
"""

kp, ki, setp = 0.06, 0.05, 8

fig, (ax1, ax2) = plt.subplots(1, 2, num = 'barrido en D', figsize = (10, 4))

kds = np.linspace(0, 0.05, 21)
n1, n2 = 0, 10
standard_deviations = []


for kd in kds[n1:n2]:
    filename = './20181114/pid_sin_ruido_%s_%s_%s_%s_caja_abierta_barrido_d.npy' % (kp, ki, kd, setp)
    t, m, c = np.load(filename)
    std_dev = np.std(m[t>.2])
    standard_deviations.append(std_dev)
    ax1.plot(t, m, label = '%.2f' % (kd) )

ax1.legend(ncol = 2)
ax1.set_xlabel('Tiempo [s]')
ax1.set_ylabel('Voltage en fotodiodo [V]')
ax1.grid()

ax2.plot(kps[n1:n2], standard_deviations, 'o-')
ax2.set_xlabel(r'Valores de I [V/V]')
ax2.set_ylabel('Desviación estandar [V]')
ax2.grid()

plt.tight_layout()

plt.savefig('barrido_en_D', dpi = 180)
#%%
"""
Distintos buffers: 
"""
kp = 0.06
ki = 0.05
kd = 0.003
setp = 8
for N in range(1,10):
    filename = './20181114/pid_sin_ruido_%s_%s_%s_%s_caja_abierta_conbuffer_%s.npy' % (kp, ki, kd, setp, N)
    t, m, c = np.load(filename)
    plt.plot(t, m, label = N)
plt.legend()
#%%
"""
PID con ruido
"""

sin_ruido = np.load('./20181114/pid_0_0_0_8_caja_abierta_conruido_500_mHz_mastenue.npy')
con_ruido_100_mHz = np.load('./20181114/pid_sin_ruido_0.06_0.05_0.003_8_caja_abierta_conruido_100_mHz.npy')
con_ruido_200_mHz = np.load('./20181114/pid_sin_ruido_0.06_0.05_0.003_8_caja_abierta_conruido_200_mHz.npy')
con_ruido_500_mHz = np.load('./20181114/pid_0.06_0.05_0.003_8_caja_abierta_conruido_500_mHz_mastenue.npy')
con_ruido_1_Hz = np.load('./20181114/pid_0.06_0.05_0.003_8_caja_abierta_conruido_1_Hz_mastenue.npy')
con_ruido_2_Hz = np.load('./20181114/pid_0.06_0.05_0.003_8_caja_abierta_conruido_2_Hz_mastenue.npy')
con_ruido_5_Hz = np.load('./20181114/pid_0.06_0.05_0.003_8_caja_abierta_conruido_5_Hz_mastenue.npy')
mediciones = {'Sin ruido': sin_ruido, '500mHz': con_ruido_500_mHz 
              ,'1Hz': con_ruido_1_Hz, '2Hz': con_ruido_2_Hz, '5Hz': con_ruido_5_Hz}


sin_ruido[1] = sin_ruido[1] * 4 - 8 # esto es un truco ;)



noise = np.random.rand(len(con_ruido_2_Hz[0]))
con_ruido_2_Hz[1] = 2*con_ruido_2_Hz[1] -8
#con_ruido_2_Hz[2] = con_ruido_2_Hz[2] + noise
con_ruido_5_Hz[1] = 4*con_ruido_5_Hz[1] -24

fig, axs = plt.subplots(5, 2, sharex=True, num ='Con ruido', figsize = (8, 6))

i = 0
n = 10
for med in mediciones:

    #ax1.cla(), ax2.cla()
    t, m, c = mediciones[med]
    axs[i][1].plot(t[n:], c[n:], '-')
    axs[i][0].plot(t[n:], m[n:], '-')
    axs[i][0].set_ylabel(med)
    
    amplitud = np.max(m[30:])-np.min(m[30:])
    print(med, '%.2f' % (amplitud) )
    i = i + 1
    


axs[0][0].set_title('Voltaje medido en fotodiodo')
axs[0][1].set_title('Salida del PID')


fig.text(0.5, 0.02, 'Tiempo [s]', ha='center')
fig.text(0, 0.5, 'Voltaje [V]', va='center', rotation='vertical')

fig.tight_layout(rect=[0, 0.03, 1, 0.95])

#plt.savefig('con_ruido_externo.pdf', dpi = 180)

#%%
plt.close('all')

