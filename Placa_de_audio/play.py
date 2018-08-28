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

def playrec_delay(freq = 220, tiempo = 3,fs=44100):
    """
    mide el tiempo entre la salida y la entrada de playrec_tone
    con salto distinto de 1 se puede acelerar la medicion pero se sobreestima un poco
    """
    a,b,c = playrec_tone(freq,tiempo)
    det = True
    i = 0
    threshold = np.mean(np.abs(c[20000:-1])) 
    while det:
        if np.abs(c[2000+i]) >= threshold/3:#al principio hay un pico, por eso los 2000
            det = False
        else:
            i = i+1
    return (i+2000)/fs

def frequency_response(points, freqstart = 100, freqend = 10000, duracion = 1):
    """
    Evalua la respuesta en frecuencia del conjunto salida y entrada de audio
    """
    response = np.zeros(points)
    for i in range(points):
        a, d, rec = playrec_tone(freqstart+i/points*(freqend-freqstart),duracion)
        response[i] = np.mean(np.abs(rec)) 
    return response
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