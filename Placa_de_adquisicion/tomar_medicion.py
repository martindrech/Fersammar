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
#from Placa_de_audio.play import generador_de_senhal, playrec_tone, play_tone, play_sawtooth
from scipy.signal import find_peaks
#%%
for device in system.devices:
    print(device)
    
    
#phys_chan = device.ai_physical_chans['ch2']
#phys_chan

task = nidaqmx.Task()
task.ai_channels.add_ai_voltage_chan('Dev1/ai0',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)
#task.ai_channels.add_ai_voltage_chan('Dev1/ai1',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)


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


#np.save('D:\\Intrumentacion_fersammar\\20181017\\num_puntos_y_len_idx_arreglado.npy', [num_puntos,len_idx])
#%%
play_tone(200,5,fs=44100,wait=False)
time.sleep(1)
with nidaqmx.Task() as read_task:

    read_task.ai_channels.add_ai_voltage_chan('Dev1/ai0',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)
    read_task.timing.cfg_samp_clk_timing(20000) # seteo la frecuencia de muestreo
    a = read_task.read(512)

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
#%% 
def measure_sample_time_2_channels(frec_de_rampa, frecuencia_de_muestreo, fs=44100):
    """
    esto es para medir tiempo entre puntos medidos adyacentes
    """ 
    duracion = 1024/frecuencia_de_muestreo+2
    play_sawtooth(frec_de_rampa, duracion, fs=44100, wait=False)
    time.sleep(1)
    with nidaqmx.Task() as read_task:
        read_task.ai_channels.add_ai_voltage_chan('Dev1/ai0',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)
        read_task.timing.cfg_samp_clk_timing(20000) # seteo la frecuencia de muestreo
        a = read_task.read(1024//len(read_task.channel_names))
        
        a0, a1 = np.array(a[0]), np.array(a[1])
        plt.figure('plot de puntos ady')
        plt.clf()
        diferencias = np.diff(a0)
        plt.plot(a0[1:-1]-a1[0:-2])
        plt.plot(a0)
        dt_sample = 1/(2*3*frec_de_rampa) * np.mean(np.abs(diferencias))
        return dt_sample*1e6

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
     

#%% 
def measure_sample_time_1_channel(frec_de_rampa, frecuencia_de_muestreo, fs=44100):
    """
    esto es para medir tiempo entre puntos medidos adyacentes de un solo canal
    Esta funcion devuelve el tiempo de adquisicon en un canal en us
    fs: Frecuencia de muestreo de la placa de audio para la funcion play_sawtooth
    """
    duracion = 1024/frecuencia_de_muestreo+2
    play_sawtooth(frec_de_rampa, duracion, fs=44100, wait=False)
    time.sleep(1)
    with nidaqmx.Task() as read_task:
        read_task.ai_channels.add_ai_voltage_chan('Dev1/ai0',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)
        read_task.timing.cfg_samp_clk_timing(frecuencia_de_muestreo) # seteo la frecuencia de muestreo
        a = read_task.read(1024//len(read_task.channel_names))

    
    plt.figure('plot de puntos ady')
    plt.clf()
    diferencias = np.diff(a)
    #plt.hist(np.abs(diferencias))
    plt.plot(np.abs(diferencias)/np.max(diferencias))
    plt.plot(a/np.max(a))
    dt_sample = 1/(2*3*frec_de_rampa) * np.mean(np.abs(diferencias)) #en us
    std_sample = 1/(2*3*frec_de_rampa) * np.std(np.abs(diferencias)) #en us
    
    return dt_sample*1e6, std_sample*1e6

#fig, (ax1, ax2) = plt.subplots(1, 2, num = 'rampa y difs')
#ax1.plot(a0)
#ax2.plot(np.diff(a))

#np.save('D:\\Intrumentacion_fersammar\\20181017\\rampa_a_500hz_2_canales.npy', a)
#%%
medios = []
errores=[]
i=0
while i<4:   
    time.sleep(1)
    with nidaqmx.Task() as write_task:
        write_task.ao_channels.add_ao_voltage_chan('Dev1/ao0', min_val=0, max_val=5)
        write_task.write(i)
    with nidaqmx.Task() as read_task:
        read_task.ai_channels.add_ai_voltage_chan('Dev1/ai1',terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)
        read_task.timing.cfg_samp_clk_timing(1000) # seteo la frecuencia de muestreo
        a = read_task.read(1024//len(read_task.channel_names))
        medios.append(np.mean(a))
        errores.append(np.std(a))
    i=i+0.5
        #a0, a1 = np.array(a[0]), np.array(a[1])
        #plt.figure('plot de puntos ady')
        #plt.clf()
        #diferencias = np.diff(a0)
        #plt.plot(a)
        #plt.plot(a0)
#%%
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
'''   
i=0
while i<4:   
    time.sleep(1)
    write_to_daq(i)
    m = read_daq()
    i=i+0.5
'''    

        
def loop_P(P,setpoint,duracion):
    write_to_daq(0)
    current_value = 0
    start_time = time.time()
    medicion, tiempo = [],[]
    plt.figure()
    plt.ion()
    plt.grid()
    loopcheck = True
    while loopcheck:
        m = float(np.array(read_daq()))
        tiempo.append(time.time()-start_time)
        medicion.append(m)
        error = m-setpoint
        write_to_daq(current_value-P*error)
        current_value -= P*error
        plt.plot(time.time()-start_time,m,'r.-')
        plt.pause(0.1)
        if (time.time()-start_time)>duracion:
            loopcheck = False
    return tiempo, medicion
        
def loop_PI(P,I,setpoint,duracion):
    write_to_daq(0)
    current_value = 0
    start_time = time.time()
    medicion, tiempo = [0],[0]
    S = 0
    plt.figure()
    plt.ion()
    plt.grid()
    loopcheck = True
    while loopcheck:
        m = float(np.array(read_daq()))
        tiempo.append(time.time()-start_time)
        medicion.append(m)
        error = m-setpoint
        S =+ ((m+medicion[-2])/2)*(tiempo[-1]-tiempo[-2])
        write_to_daq((abs(current_value-P*error-I*S)+(current_value-P*error-I*S))/2)
        current_value -= P*error
        plt.plot(time.time()-start_time,m,'r.-')
        plt.pause(0.1)
        if (time.time()-start_time)>duracion:
            loopcheck = False
    return tiempo, medicion

#def loop_PID(P,I,D,setpoint,duracion): #vieja
#    write_to_daq(0)
#    current_value = 0
#    start_time = time.time()
#    medicion, tiempo = [0],[0]
#    S = 0
#    plt.figure()
#    plt.ion()
#    plt.grid()
#    loopcheck = True
#    while loopcheck:
#        measured_value = float(np.array(read_daq(sample_rate=10000)))
#        tiempo.append(time.time()-start_time)
#        medicion.append(m)
#        error = m-setpoint
#        S =+ ((m+medicion[-2])/2)*(tiempo[-1]-tiempo[-2])
#        derivada = (medicion[-1]-medicion[-2])/(tiempo[-2]-tiempo[-1])
#        try:
#            write_to_daq(current_value-P*error-I*S-D*derivada)
#        except:
#            write_to_daq((np.sign(current_value-P*error-I*S-D*derivada)+1)*2.5)
#        current_value -= P*error
#        plt.plot(time.time()-start_time,m,'r.-')
#        plt.pause(0.1)
#        if (time.time()-start_time)>duracion:
#            loopcheck = False
#    return tiempo, medicion  

def start_loop_PID(P,I,D,setpoint,duracion):
    write_to_daq(0)
    PID = PID_loop(P, I, D, setpoint)
    start_time = time.time()
    
    plt.figure()
    plt.ion()
    plt.grid()
    loopcheck = True
    
    while loopcheck:
        measured_value = float(np.array(read_daq(sample_rate=10000)))
        PID.add_medicion(measured_value, time.time()-start_time)
        PID.calc_parameters(measured_value)
        
        new_actuator_value = pid.calculate(signal)
        
        try:
            write_to_daq(PID.value_to_write)
        except:
            write_to_daq((np.sign(PID.value_to_write)+1)*2.5)
            
        PID.update_current_value()
        plt.plot(time.time()-start_time, measured_value, 'r.-')
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
        
    def calculate_new(self, m, t):
        
"""
class LazoPID:
    
    def __init__(self, setpoint, p, i, d):
        #
        
    def __call__(self, signal):
        self.i_value += signal
        return self.p*(signal - self.setpoint)
        
    
    
def create_pid(setpoint, p, i, d):
    
    def _internal():
        
        while True:
            # <- señal
            # -> correccion
            signal = yield correction
            correction = p*(signal - setpoint)
    
    gen = internal()
    gen.send(None)
    return gen


lazo = LazoPID(3, 2, 6, 3)

while True:
    
    daq.write(lazo(daq.read()))
    
"""       
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        