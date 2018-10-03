# -*- coding: utf-8 -*-
"""
Editor de Spyder

Este es un archivo temporal.
"""

import nidaqmx
import time
import numpy as np
from matplotlib import pyplot as plt
import nidaqmx.system
system = nidaqmx.system.System.local()
system.driver_version
import sounddevice as sd

for device in system.devices:
    print(device)
    
    
phys_chan = device.ai_physical_chans['ch1']
phys_chan


task = nidaqmx.Task()
task.ai_channels.add_ai_voltage_chan('Dev1/ai0')
#%%
medicion = []
#tiempo = []
start_time =time.time()
while True:
    data = task.read()
    #print(data)
    medicion.append(data)
    #tiempo.append(time.time())
    #time.sleep(0.2)
    if (time.time()-start_time)>15:
        break

plt.plot(medicion)

#%%


from Placa_de_audio.play import generador_de_senhal, playrec_tone, play_tone

def barrido_frecuencias(tiempo, f0, fN, df):
    medicion_salida = []
    frecuencias = np.arange(f0, fN, df)
    for f in frecuencias:
        play_tone(f, tiempo, amplitud=1)
        data = task.read()
        transformada = np.fft.fft(data)
        indice_max_frecuencia = np.argmax(transformada)
        #print(data)
        medicion_salida.append(indice_max_frecuencia)
        #tiempo.append(time.time())
        #time.sleep(0.2)
    return medicion_salida

        
indice_frecuencia_salida = barrido_frecuencias(2,100,10000,100)
frecuencia_entrada = np.arange(100,10000,100)
freqs = np.fft.fftfreq(x.size)*2*np.pi/dt #obtener el dt

#Tambien se podria mandar a medir todo junto y despues analizar, para que vaya mas rapido
