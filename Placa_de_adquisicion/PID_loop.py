# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 15:37:25 2018

@author: Publico
"""

import pylab as plt
import numpy as np
from talk_to_daq import write_to_daq, read_daq
import time



def start_loop_PID(P,I,D,setpoint,duracion):
    write_to_daq(0)
    PID = PID_loop(P, I, D, setpoint)
    start_time = time.time()
    
    fig, (ax1, ax2) = plt.subplots(1, 2)
    
    plt.ion()
    ax1.grid(), ax2.grid()
    loopcheck = True
    
    
    
    while loopcheck:
        measured_value = float(np.array(read_daq(sample_rate=40000)))
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
       
    return PID.tiempo, PID.medicion              

class PID_loop:
    def __init__(self, P, I, D,setpoint):
        self.P = P
        self.D = D
        self.I = I
        self.setpoint = setpoint
        
        self.current_value = 0
        self.S = 0
        self.derivada = 0
        self.error = 0
        self.value_to_write = 0
        
        
        self.medicion = [0]
        self.tiempo = [0]
        
            
    def add_medicion(self, m, t):
        self.medicion.append(m)
        self.tiempo.append(t)
    
    def calc_parameters(self, measured_value):
        self.error = measured_value - self.setpoint
        self.S += ((measured_value+self.medicion[-2])/2)*(self.tiempo[-1]-self.tiempo[-2]) 
        self.derivada = (self.medicion[-1]-self.medicion[-2])/(self.tiempo[-2]-self.tiempo[-1])
        self.value_to_write = self.current_value-self.P*self.error-self.I*self.S-self.D*self.derivada 
        
    def update_current_value(self):
        self.current_value -= self.P*self.error
        
