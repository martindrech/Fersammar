# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 15:37:25 2018

@author: Publico
"""

import pylab as plt
import numpy as np
from talk_to_daq import write_to_daq, read_daq
import time



def start_loop_PID(P,I,D,setpoint,duracion,buffer=False, buf_size=5):
    write_to_daq(0)
    PID = PID_loop(P, I, D, setpoint,buffer, buf_size)
    start_time = time.time()
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize = (12, 6))
    
    plt.ion()
    ax1.grid(), ax2.grid()
    loopcheck = True
    
    
    
    while loopcheck:
        measured_value = float(np.array(read_daq(sample_rate=40000, num_samples=1)))
        PID.add_medicion(measured_value, time.time()-start_time)
        PID.calc_parameters(measured_value)
        
        new_actuator_value = PID.value_to_write
        
        try:
            write_to_daq(new_actuator_value)
        except:
            write_to_daq((np.sign(new_actuator_value)+1)*2.5)
            
        PID.update_current_value()
        ax1.plot(time.time()-start_time, measured_value, 'r.-')
        ax1.plot(time.time()-start_time, PID.error, 'b.-')
        ax2.plot(time.time()-start_time, PID.value_to_write, 'g.-')
        
        plt.pause(0.1)
        if (time.time()-start_time)>duracion:
            loopcheck = False
       
    return PID.tiempo, PID.medicion, PID.control_values            

class PID_loop:
    def __init__(self, P, I, D,setpoint, buffer=True, buf_size=5):
        self.P = P
        self.D = D
        self.I = I
        self.setpoint = setpoint
        
        self.current_value = 0
        self.S = 0
        self.suma_parcial = list(np.zeros(buf_size))
        self.derivada = 0
        self.error = 0
        self.value_to_write = 0
        
        
        self.medicion = [0]
        self.tiempo = [0]
        self.control_values = [0]
            
    def add_medicion(self, m, t):
        self.medicion.append(m)
        self.tiempo.append(t)
        
    
    def calc_parameters(self, measured_value, buffer=False):
            """
    Calcula la integral y la derivada del PID y devuelve el proximo valor a enviar.
    Parameters
    ----------
    measured_value : Ultimo valor medido por el fotodiodo
    buffer : Si la integral acarca todo el tiempo desde el inicio, o solo un determinado                buffer
    
    Returns
    -------
    value_to_write : Proximo valor a enviar al led
    """
        self.error = measured_value - self.setpoint
        if buffer==False:
            self.S += ((self.error+self.medicion[-2]-self.setpoint)/2)*(self.tiempo[-1]-self.tiempo[-2])
        else:
            self.suma_parcial.append(((self.error+self.medicion[-2]-self.setpoint)/2)*(self.tiempo[-1]-self.tiempo[-2]))
            self.suma_parcial.pop(0)
            self.S = np.sum(self.suma_parcial)
        self.derivada = (self.medicion[-1]-self.medicion[-2])/(self.tiempo[-1]-self.tiempo[-2])
        self.value_to_write = self.current_value-self.P*self.error-self.I*self.S-self.D*self.derivada 
        self.control_values.append(self.value_to_write)
        
    def update_current_value(self):
        self.current_value -= self.P*self.error
 
#%% k ultimate calculation
kp = 0.06
ki = 0.05
setp = 8
for kd in np.linspace(0, 0.05, 21):
    t, m, c = start_loop_PID(kp, ki, 0, setp, 10)
    filename = 'D:\\Intrumentacion_fersammar\\20181114\\pid_sin_ruido_%s_%s_%s_%s_caja_abierta_barrido_d.npy' % (kp, ki, kd, setp)
    np.save(filename, [t, m, c])
    print(kd)


#%%
plt.figure()
kps = np.linspace(0, 0.2, 20)
for kp in kps:
    filename = 'D:\\Intrumentacion_fersammar\\20181114\\pid_sin_ruido_%s_%s_%s_%s_caja_abierta_barrido_p.npy' % (kp, ki, kd, setp)
    t, m, c = np.load(filename)
    plt.plot(t, m, label = '%.2f' % (kp) )
plt.legend()
#%%
plt.figure()
kis = np.linspace(0, 0.1, 10)
for ki in kis:
    filename = 'D:\\Intrumentacion_fersammar\\20181114\\pid_sin_ruido_%s_%s_%s_%s_caja_abierta_barrido_I_sinbuffer.npy' % (kp, ki, kd, setp)
    t, m, c = np.load(filename)
    plt.plot(t, m, label = '%.2f' % (ki) )
plt.legend()
#%%
plt.figure()
kds = np.linspace(0, 0.05, 21)[:3]
for kd in kds:
    filename = 'D:\\Intrumentacion_fersammar\\20181114\\pid_sin_ruido_%s_%s_%s_%s_caja_abierta_barrido_d.npy' % (kp, ki, kd, setp)
    t, m, c = np.load(filename)
    plt.plot(t, m, label = '%.3f' % (kd) )
plt.legend()

#%%
#ya elegidos kp,ki,kd tomamos con el buffer
kp = 0.06
ki = 0.05
kd = 0.003
setp = 8
for N in range(1,10):
    t, m, c = start_loop_PID(kp, ki, kd, setp, 10,buffer=True, buf_size=N)
    filename = 'D:\\Intrumentacion_fersammar\\20181114\\pid_sin_ruido_%s_%s_%s_%s_caja_abierta_conbuffer_%s.npy' % (kp, ki, kd, setp, N)
    np.save(filename, [t, m, c])
#%%

frec_ruido = 5

t, m, c = start_loop_PID(kp, ki, kd, setp, 10,buffer=False)
filename = 'D:\\Intrumentacion_fersammar\\20181114\\pid_%s_%s_%s_%s_caja_abierta_conruido_%s_Hz_mastenue.npy' % (kp, ki, kd, setp, frec_ruido)
np.save(filename, [t, m, c])    

#%%
ku = 0.92
Tu = 0.32
kp = .6*ku
ki = 2*kp/Tu
kd = kp*Tu/8
t, m = start_loop_PID(kp, ki, kd, 4, 30)

#%% medicion larga
kp = 0.06
ki = 0.05
kd = 0 
t, m, c = start_loop_PID(kp, ki, kd, setp, 1800)
filename = 'D:\\Intrumentacion_fersammar\\20181114\\pid_sin_ruido_%s_%s_%s_%s_caja_abierta_med_larga.npy' % (kp, ki, kd, setp)
np.save(filename, [t, m, c])