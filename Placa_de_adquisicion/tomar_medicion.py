# -*- coding: utf-8 -*-
"""
Editor de Spyder

Este es un archivo temporal.
"""

import nidaqmx
from nidaqmx.stream_readers import AnalogSingleChannelReader
import time
import numpy as np
from matplotlib import pyplot as plt
import nidaqmx.system
system = nidaqmx.system.System.local()
system.driver_version
import sounddevice as sd
from Placa_de_audio.play import generador_de_senhal, playrec_tone, play_tone, play_sawtooth
from scipy.signal import find_peaks

for device in system.devices:
    print(device)
    
    
#phys_chan = device.ai_physical_chans['ch2']
#phys_chan

task = nidaqmx.Task()
task.ai_channels.add_ai_voltage_chan('Dev1/ai0',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)
task.ai_channels.add_ai_voltage_chan('Dev1/ai1',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)


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
num_puntos = np.arange(100,512,10)
#num_puntos = [1023]
len_idx = []
frec_de_calibracion = 500
play_tone(frec_de_calibracion,30,fs=44100,wait=False)
time.sleep(3)
for n in num_puntos:
    print(n)
    a = task.read(n)[1]
    idx = find_peaks(a, distance = 2)
    len_idx.append(len(idx[0]))
sd.stop()
#plt.plot(a)
plt.plot(num_puntos,len_idx,'.-')
m, b = np.polyfit(num_puntos, len_idx, 1)
plt.plot(num_puntos, num_puntos*m+b, 'r')
t_sample = m * 1/frec_de_calibracion
print(t_sample *1e6, ' us')


np.save('D:\\Intrumentacion_fersammar\\20181017\\num_puntos_y_len_idx_arreglado.npy', [num_puntos,len_idx])
#%%
play_tone(200,5,fs=44100,wait=False)
time.sleep(1)

a = task.read(512)[0]

plt.figure('un plot')
plt.clf()
plt.plot(a, '.-')
idx, props = find_peaks(a, distance = 50)
print(idx)
dt = ( (1/200)/(idx[1]-idx[0]) )
#f0 = find_freq(dt, a)
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
    frecuencias = np.arange(f0, fN, df)
    #frecuencias = [6000]
    for f in frecuencias:
        print(f)
        play_tone(f, tiempo,fs=44100, amplitud=1, wait=False)
        time.sleep(1)
        data = task.read(512)[0]
        data = data - np.mean(data)
        plt.figure('data')
        plt.plot(data)
        medicion_salida.append(find_freq(12.82e-05, data, should_I_plot = True))
        #tiempo.append(time.time())
        #time.sleep(0.2)
        sd.stop()
        
    return frecuencias, medicion_salida

        
frecuencia_entrada, frecuencia_salida = barrido_frecuencias(5,100,10000,50)

plt.figure('frec de entrada y de salida')
plt.clf()
plt.plot(frecuencia_entrada, frecuencia_salida, 'o')
plt.xlabel('frecuencia de entrada (Hz)')
plt.ylabel('frecuencia de salida (Hz)')
plt.grid()
#%%
a = task.read()
print(task.in_stream.avail_samp_per_chan())
#%% esto es para medir tiempo entre puntos medidos adyacentes
frec_de_rampa = 500
play_sawtooth(frec_de_rampa, 5, fs=44100, wait=False)
time.sleep(1)
a = task.read(1024//len(task.channel_names))
a0, a1 = np.array(a[0]), np.array(a[1])
plt.figure('plot de puntos ady')
plt.clf()
diferencias = np.diff(a0)
plt.plot(a0[1:-1]-a1[0:-2])
plt.plot(a0)
dt_sample = 1/(2*3*frec_de_rampa) *0.1# np.mean(np.abs(diferencias))
print(dt_sample*1e6, ' us')

#fig, (ax1, ax2) = plt.subplots(1, 2, num = 'rampa y difs')
#ax1.plot(a0)
#ax2.plot(np.diff(a))

#np.save('D:\\Intrumentacion_fersammar\\20181017\\rampa_a_500hz_2_canales.npy', a)

#%%
data_values = np.zeros(100)
with nidaqmx.Task() as read_task:
     read_task.ai_channels.add_ai_voltage_chan('Dev1/ai0',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)
     reader = AnalogSingleChannelReader(read_task.in_stream)
     print(read_task.in_stream.input_buf_size)
     value_read = reader.read_many_sample(data_values, 100)
     
