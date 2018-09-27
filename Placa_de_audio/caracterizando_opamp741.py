#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 20 15:50:39 2018


Los primeros bloques son para tomar los datos. Los ultimos para analizar la data, y hay en donda este este script hay que poner la carpeta mediciones_opamp.

@author: liaf
"""

import play as play
import pylab as plt
import numpy as np


#%% caraterziacion en frecuencia del circuito plado
"""
f, A = frequency_response(100, freqstart=100, freqend=100000)
np.savetxt('respuesta_frecuencia.txt', np.c_[f, A])
"""

#%% esto es para levantar respuesa en frecuencia de distintos circuitos

t, d, g = play.playrec_tone(30000, .1, amplitud=0.2)
g1, g2 = g[:, 0], g[:, 1]
n = 500
n0 = len(t)
t, d, g1, g2 = t[-n:-1], d[-n:-1], g1[-n:-1],g2[-n:-1]
plt.plot(t, g1, label = 'g1')
plt.plot(t, g2, label = 'g2')
plt.legend()
#%% aca se puede guardar la data
f, A = play.frequency_response(50, freqstart=100, freqend=100000)
#np.savetxt('respuesta_frecuencia_opamp_C.txt', np.c_[f, A])

#%% igual que antes pero con una cuadrada
t, d, g = play.playrec_square(50000, .2, amplitud=0.2)
g1, g2 = g[:, 0], g[:, 1]
n = 50
n0 = len(t)
t, d, g1, g2 = t[-n:-1], d[-n:-1], g1[-n:-1],g2[-n:-1]
plt.plot(t, g1, label = 'g1')
plt.plot(t, g2, label = 'g2')
plt.legend()
#%% ahora cargo data que ya hay guardada
data = np.loadtxt('mediciones_opamp/respuesta_frecuencia_opamp_R.txt')
f_R, A_R = data[:, 0], data[:, 1]
data = np.loadtxt('mediciones_opamp/respuesta_frecuencia_opamp_C.txt')
f_C, A_C= data[:, 0], data[:, 1]
data = np.loadtxt('mediciones_opamp/respuesta_frecuencia_opamp_RC.txt')
f_RC, A_RC= data[:, 0], data[:, 1]

data = np.loadtxt('mediciones_opamp/respuesta_frecuencia.txt')
f, A= data[:, 0], data[:, 1]

#%% ploteo
plt.figure('respuestas en frecuencia')
plt.clf()
plt.plot(f_RC, A_RC, '.-',label ='RC')
plt.plot(f_R, A_R, '.-',label ='R')
plt.plot(f_C, A_C, '.-',label ='C')
plt.legend()
#%% defino funcion de transferencia (teorica y ploteo para comparar)
def T_function(f):
    w = 2*np.pi*f
    wc = 1/(22e3*1e-9)
    T = 1/(1+1j*w/wc)
    return np.abs(T)

plt.figure('filtro pasabajo: comparacion con teoria')
plt.clf()
plt.loglog(f_RC *1e-3, A_RC, '.-',label ='Medición')
plt.loglog(f_RC *1e-3, T_function(f_RC) * A_RC[0], label = 'Teoría')
plt.legend(), plt.xlabel('f [kHz]'), plt.grid()
plt.axis([0, 50, 0, 0.21])
#%% ploteo de la respuesta del filtro a una entrada cuadrada

data = np.loadtxt('respuesta_a_cuadrada_20khz.txt')
t, d, ch2, ch1 = np.split(data, 4, 1)
t = t * 1e3
plt.clf()
plt.plot(t, ch1, label = 'ch1')
plt.plot(t, -ch2, label = 'ch2')
plt.legend(), plt.grid()
plt.xlabel('t [ms]')
#plt.axis([198.6, 199.2, -.45, .45])
