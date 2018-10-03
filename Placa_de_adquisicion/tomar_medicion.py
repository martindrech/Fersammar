# -*- coding: utf-8 -*-
"""
Editor de Spyder

Este es un archivo temporal.
"""

import nidaqmx
import time
from matplotlib import pyplot as plt
import nidaqmx.system
system = nidaqmx.system.System.local()
system.driver_version

for device in system.devices:
    print(device)
    
    
phys_chan = device.ai_physical_chans['ch1']
phys_chan


task = nidaqmx.Task()
task.ai_channels.add_ai_voltage_chan('Dev1/ai0')
#%%
medicion = []
while True:
    data = task.read()
    print(data)
    medicion.append(data)
    time.sleep(0.2)
    if len(medicion)>100:
        break

#%%

plt.plot(medicion)

#ch = task._ai_channels.add_ai_voltage_chan()