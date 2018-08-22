import sounddevice as sd
import numpy as np
import pylab as plt
import time

def constant(amplitud, duracion,  fs=44100):
    sd.default.samplerate = fs #frecuencia de muestreo
    #x = np.sin(np.linspace(amplitud-0.01, amplitud+0.01, fs*duracion))
    #x = amplitud*np.ones(fs*duracion)+10
    tiempo = 5*np.linspace(0, duracion, 220000)
    
    x = .2*np.sin(2*np.pi*220*tiempo) +.5*np.ones(len(tiempo))
    sd.play(x)
    
def play_tone(f, duracion, fs=44100, wait=True):
    sd.default.samplerate = fs #frecuencia de muestreo
        
    cantidad_de_periodos = duracion*f
    puntos_por_periodo = int(fs/f)
    puntos_totales = puntos_por_periodo*cantidad_de_periodos
           
    tiempo = np.linspace(0, duracion, puntos_totales)
    
    data = np.sin(2*np.pi*f*tiempo)#+np.sin(2*np.pi*f*1.5*x)+np.sin(2*np.pi*f*x*2) 
       
    
    sd.play(data)
    
    if wait:
        time.sleep(duracion)
        
    return puntos_totales
    
#%%
tones = np.array([165, 175, 196, 220, 247, 262, 294, 330])*.002
for i in tones:
    play_tone(i, 1)
    time.sleep(1)
    print(i)
#%%
constant(3,5)