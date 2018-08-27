import sounddevice as sd
import numpy as np
import pylab as plt
import time
 
def play_tone(frecuencia, duracion, fs=44100, wait=True):
    """
    Esta función tiene como output un tono de una cierta duración y frecuencia.
    """
    sd.default.samplerate = fs #frecuencia de muestreo
    
    #Estas 3 lineas siguientes se podrian reducir
    cantidad_de_periodos = duracion*frecuencia
    puntos_por_periodo = int(fs/frecuencia)
    puntos_totales = puntos_por_periodo*cantidad_de_periodos
           
    tiempo = np.linspace(0, duracion, puntos_totales)
    
    data = np.sin(2*np.pi*frecuencia*tiempo)
       
    sd.play(data)
    
    if wait:
        time.sleep(duracion)
        
    return puntos_totales

def test_play_tone():
    """
    Test para ver si funciona play_tone, tocando varios tonos.
    """
    tones = np.array([165, 175, 196, 220, 247, 262, 294, 330])
    for i in tones:
        play_tone(i, 1)
        time.sleep(1)
        print(i)

def playrec_tone(frecuencia, duracion, fs=44100):
    """
    Emite un tono y lo graba.
    """
    sd.default.samplerate = fs #frecuencia de muestreo
    sd.default.channels = 1,2 #por las dos salidas de audio
    
    cantidad_de_periodos = duracion*frecuencia
    puntos_por_periodo = int(fs/frecuencia)
    puntos_totales = puntos_por_periodo*cantidad_de_periodos
           
    tiempo = np.linspace(0, duracion, puntos_totales)
    
    data = np.sin(2*np.pi*frecuencia*tiempo)
       
    grabacion = sd.playrec(data, blocking=True)
    return tiempo, data, grabacion

def test_playrec_tone():
    """
    Test para ver si funciona playrec_tone, emitiendo y grabando varios tonos.
    """
    tones = np.array([165, 175, 196, 220, 247, 262, 294, 330])
    for i in tones:
        tiempo, data, grabacion = playrec_tone(i, 1)
        time.sleep(1)
        print(i)
    return grabacion

def record(duracion, fs=44100):
    """
    Graba la entrada de microfono por el tiempo especificado
    """
    sd.default.samplerate = fs #frecuencia de muestreo
    sd.default.channels = 1 #1 porque la entrada es una sola
    
    grabacion = sd.rec(frames = fs*duracion, blocking = True)
    return grabacion
#%%
def constant(amplitud, duracion,  fs=44100):
    """
    La placa de audio no está hecha para emitir una constante.
    """
    sd.default.samplerate = fs #frecuencia de muestreo
    #x = np.sin(np.linspace(amplitud-0.01, amplitud+0.01, fs*duracion))
    #x = amplitud*np.ones(fs*duracion)+10
    tiempo = 5*np.linspace(0, duracion, 220000)
    
    x = .2*np.sin(2*np.pi*220*tiempo) +.5*np.ones(len(tiempo))
    sd.play(x)