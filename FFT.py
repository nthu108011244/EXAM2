import matplotlib.pyplot as plt
import numpy as np
import serial
import time

Fs = 10;  # sampling rate
Ts = 1; # sampling interval
t = np.arange(0,10,Ts) # time vector; create Fs samples between 0 and 1.0 sec.
y = np.arange(0,10,Ts) # signal vector; create Fs samples
Y = np.arange(0,10,Ts) # signal vector; create Fs samples

n = len(y) # length of the signal
k = np.arange(n)
T = n/Fs

serdev = '/dev/ttyACM0'
s = serial.Serial(serdev)
for x in range(0, int(Fs)):
    line=s.readline() # Read an echo string from B_L4S5I_IOT01A terminated with '\n'
    # print line
    y[x] = int(line)
for x in range(0, int(Fs)):
    line=s.readline() # Read an echo string from B_L4S5I_IOT01A terminated with '\n'
    # print line
    Y[x] = int(line)


fig, ax = plt.subplots(1, 1)
ax[0].plot(t,y)
ax[0].set_xlabel('Time')
ax[0].set_ylabel('Amplitude')
ax[1].plot(t,Y,'r') # plotting the spectrum
ax[1].set_xlabel('Freq (Hz)')
ax[1].set_ylabel('|Y(freq)|')
plt.show()
s.close()