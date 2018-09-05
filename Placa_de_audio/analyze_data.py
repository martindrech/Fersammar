#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  5 13:36:09 2018

@author: martindrech
"""

import numpy as np
import pylab as plt

def find_index_of_nearest(array, value):
    """
    Funcion auxiliar que encuentra el indice del elemento mas cercano a cierto valor en un array
    """
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx


data_100hz = np.loadtxt('datos_diodo_100hz.txt')
data_200hz = np.loadtxt('datos_diodo_200hz.txt')
data_300hz = np.loadtxt('datos_diodo_300hz.txt')
data_400hz = np.loadtxt('datos_diodo_400hz.txt')
data_500hz = np.loadtxt('datos_diodo_500hz.txt')
data_600hz = np.loadtxt('datos_diodo_600hz.txt')
data_700hz = np.loadtxt('datos_diodo_700hz.txt')
data_800hz = np.loadtxt('datos_diodo_800hz.txt')
data_900hz = np.loadtxt('datos_diodo_900hz.txt')

#%%

def analyze_data(datatxt, datatxt_name, n1 = 0, n2 = 80000, n_start_t = 0, n_end_t = 0, n_sat_t = 0, do_fit = False):
    """
    Analiza la data del archivo  datatxt que son las curvas del diodo a distintas frecuencias. 
    n1, n2: son los indices para plotear y mirar
    n_start, n_end: son los tiempos donde empieza y termina la regio donde quiero 
        la curva de voltaje (para corregir lo que se saturo)
    n_sat: desde n_start_t hasta aca uso para el fit de la lineal
    """
    t, d, g0, g1 = datatxt[:, 0], datatxt[:, 1], datatxt[:, 2], datatxt[:, 3]
    g0, g1 = g0*.55, g1*.55
    plt.figure('data from ' + str(datatxt_name))
    plt.clf()
    #n1 =88850
    #n2 = 90500
    #plt.plot(t[n1:n2], d[n1:n2], 'b')
    plt.plot(t[n1:n2], g0[n1:n2], label = 'V_R (g0)')
    plt.plot(t[n1:n2], g1[n1:n2], label = 'V_source (g1)')
    plt.grid()
    plt.legend()
    if do_fit:
        n_start = find_index_of_nearest(t, n_start_t)
        n_sat = find_index_of_nearest(t, n_sat_t)
        n_end = find_index_of_nearest(t, n_end_t)
        a, b = np.polyfit(t[n_start:n_sat], g1[n_start:n_sat], 1)
        fit_voltage = a*t[n_start:n_end]+b
        plt.figure('data and fit from ' + str(datatxt_name))
        plt.clf()
        plt.plot(t[n_start:n_end], fit_voltage, label = 'fit')
        plt.plot(t[n1:n2], g0[n1:n2], label = 'V_R (g0)')
        plt.plot(t[n1:n2], g1[n1:n2], label = 'V_source (g1)')
        plt.grid()
        plt.legend()
        
        plt.figure('I vs V ' + str(datatxt_name) )
        plt.clf()
        I = g0[n_start:n_end]/1000
        plt.plot(fit_voltage, I*1000)
        plt.xlabel('V [volts]')
        plt.ylabel('I [mA]')
        plt.grid()
#%%
analyze_data(data_100hz, '100hz0', do_fit=True, n1 =88850 , n2 =90500 , n_start_t=0.4637, n_end_t=0.4696, n_sat_t = 0.4668)    
analyze_data(data_300hz, '300hz0', do_fit=True, n1 =89500 , n2 =90500 , n_start_t=0.4690, n_end_t=0.4710, n_sat_t = 0.4698) 
analyze_data(data_500hz, '500hz0', do_fit=True, n1 =89800 , n2 =90200 , n_start_t=0.4683, n_end_t=0.4696, n_sat_t = 0.4690) 
analyze_data(data_900hz, '900hz0', do_fit=True, n1 =89800 , n2 =90200 , n_start_t=0.4691, n_end_t=0.4697, n_sat_t = 0.4694) 
#%%
plt.figure()
datatxt = data_100hz
t, d, g0, g1 = datatxt[:, 0], datatxt[:, 1], datatxt[:, 2], datatxt[:, 3]
plt.plot(t, d, t, g1)
plt.plot(t, g0)
        
plt.grid()
