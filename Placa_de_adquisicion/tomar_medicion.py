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
from Placa_de_audio.play import generador_de_senhal, playrec_tone, play_tone
from scipy.signal import find_peaks

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
num_puntos = np.arange(100,1024,5)
#num_puntos = [1023]
len_idx = []
play_tone(1000,30,fs=44100,wait=False)
time.sleep(3)
for n in num_puntos:
    print(n)
    a = task.read(n)
    idx = find_peaks(a, distance = 2)
    len_idx.append(len(idx[0]))
sd.stop()
#plt.plot(a)
plt.plot(num_puntos,len_idx,'.-')
m, b = np.polyfit(num_puntos, len_idx, 1)
plt.plot(num_puntos, num_puntos*m+b, 'r')




#np.save('D:\\Intrumentacion_fersammar\\20181010\\num_puntos_y_len_idx.npy', [num_puntos,len_idx])
#%%
play_tone(200,5,fs=44100,wait=False)
time.sleep(1)
a = task.read(1024)
plt.figure('un plot')
plt.clf()
plt.plot(a, '.-')
idx, props = find_peaks(a, distance = 50)
print(idx)
dt = ( (1/200)/(idx[1]-idx[0]) )
f0 = find_freq(dt, a)
#%%



def find_freq(dt, x, should_I_plot = False):
    """
    Transforma fourier y devuelve el maximo de la transformada
    """
    x=np.asarray(x)
    #dt = t[1]-t[0]
    f = np.fft.fft(x)
    freqs = np.fft.fftfreq(x.size)/dt

    f = f[:x.size//2]
    freqs = freqs[:x.size//2]
    if should_I_plot: 
        plt.figure('fft')
        plt.plot(freqs, np.abs(f), 'o')
        plt.show()

    return freqs[np.argmax(np.abs(f[1:]))+1]

def barrido_frecuencias(tiempo, f0, fN, df):
    medicion_salida = []
    #frecuencias = np.arange(f0, fN, df)
    frecuencias = [6000]
    for f in frecuencias:
        print(f)
        play_tone(f, tiempo,fs=44100, amplitud=1, wait=False)
        time.sleep(1)
        data = task.read(1024)
        data = data - np.mean(data)
        plt.figure('data')
        plt.plot(data)
        medicion_salida.append(find_freq(7.57e-05, data, should_I_plot = True))
        #tiempo.append(time.time())
        #time.sleep(0.2)
        sd.stop()
        
    return frecuencias, medicion_salida

        
frecuencia_entrada, frecuencia_salida = barrido_frecuencias(5,100,10000,100)

plt.figure('frec de entrada y de salida')
plt.clf()
plt.plot(frecuencia_entrada, frecuencia_salida, 'o')
plt.xlabel('frecuencia de entrada (Hz)')
plt.ylabel('frecuencia de salida (Hz)')
plt.grid()
