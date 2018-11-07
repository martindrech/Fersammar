# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 15:38:24 2018

@author: Publico
"""

import nidaqmx
import numpy as np
import pylab as plt
import time

import nidaqmx.system
system = nidaqmx.system.System.local()
system.driver_version


def write_to_daq(voltage):
    with nidaqmx.Task() as write_task:
        write_task.ao_channels.add_ao_voltage_chan('Dev1/ao0', min_val=0, max_val=5)
        write_task.write(voltage)
    return 0

def read_daq(sample_rate=1000,num_samples=1024):
    medicion = []
    with nidaqmx.Task() as read_task:
        read_task.ai_channels.add_ai_voltage_chan('Dev1/ai1',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)
        read_task.timing.cfg_samp_clk_timing(sample_rate) # seteo la frecuencia de muestreo
        a = read_task.read(num_samples//len(read_task.channel_names))
        medicion.append(np.mean(a))
        return medicion
    
def continous_measurement():
    #plt.figure()
    #plt.grid()
    #plt.ion()
    start_time = time.time()

    value = read_daq(sample_rate=40000)
    fig, ax = plt.subplots(1, 1 )
    line, = ax.plot(time.time()-start_time, value, '.b')    

    ax.set_xlim(0, 10)
    #times = [time.time()-start_time]
    times = [0]
    values = [value]
    while plt.fignum_exists(fig.number):
        values.append(read_daq(sample_rate=40000))
        times.append(times[-1] + 1)
        #times.append(time.time()-start_time)
        line.set_data(times, values)
        ax.set_ylim(min(np.array(values)), max(np.array((values))))
        ax.set_xlim(times[-1]-10, times[-1])
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.05)
        