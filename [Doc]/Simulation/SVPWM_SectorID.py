import matplotlib.pyplot as plt
import numpy as np

theta = np.linspace(0, np.deg2rad(360), 500)
angles = [np.deg2rad(60), np.deg2rad(120), np.deg2rad(180), np.deg2rad(240), np.deg2rad(300), np.deg2rad(360)]
anglelabels = ['60', '120', '180', '240', '300', '360']

X = np.sin(theta)
Y = (np.sin(theta) + np.sqrt(3)*np.cos(theta))/2
Z = (np.sin(theta) - np.sqrt(3)*np.cos(theta))/2

#A = -Z + 0
#A[theta>np.deg2rad(60)] = z0
#A[theta>np.deg2rad(120)] = Y[theta>np.deg2rad(120)]
#A[theta>np.deg2rad(180)] = -Z[theta>np.deg2rad(180)]
#A[theta>np.deg2rad(240)] = z0
#A[theta>np.deg2rad(300)] = Y[theta>np.deg2rad(300)]

#B = theta + 0
#B[theta>0] = 0
#B[theta>np.deg2rad(60)] = Z[theta>np.deg2rad(60)]
#B[theta>np.deg2rad(120)] = X[theta>np.deg2rad(120)]
#B[theta>np.deg2rad(180)] = z0
#B[theta>np.deg2rad(240)] = Z[theta>np.deg2rad(240)]
#B[theta>np.deg2rad(300)] = Y[theta>np.deg2rad(300)]

#C = -X + 0
#C[theta>np.deg2rad(60)] = -Y[theta>np.deg2rad(60)]
#C[theta>np.deg2rad(120)] = z0
#C[theta>np.deg2rad(180)] = -X[theta>np.deg2rad(180)]
#C[theta>np.deg2rad(240)] = -Y[theta>np.deg2rad(240)]
#C[theta>np.deg2rad(300)] = z0

z0 = (1 + Z + X)/2
As1 = z0 - Z
Bs1 = z0 
Cs1 = z0 - X

As1[theta>np.deg2rad(60)] = 0
Bs1[theta>np.deg2rad(60)] = 0
Cs1[theta>np.deg2rad(60)] = 0

#As1 = np.ma.masked_where(theta>np.deg2rad(60), As1)
#Bs1 = np.ma.masked_where(theta>np.deg2rad(60), Bs1)
#Cs1 = np.ma.masked_where(theta>np.deg2rad(60), Cs1)

z0 = (1 - Z + Y)/2
As2 = z0
Bs2 = z0 + Z 
Cs2 = z0 - Y

As2[theta<np.deg2rad(60)] = 0
As2[theta>np.deg2rad(120)] = 0
Bs2[theta<np.deg2rad(60)] = 0
Bs2[theta>np.deg2rad(120)] = 0
Cs2[theta<np.deg2rad(60)] = 0
Cs2[theta>np.deg2rad(120)] = 0

z0 = (1 - X - Y)/2
As3 = z0 + Y
Bs3 = z0 + X 
Cs3 = z0

As3[theta<np.deg2rad(120)] = 0
As3[theta>np.deg2rad(180)] = 0
Bs3[theta<np.deg2rad(120)] = 0
Bs3[theta>np.deg2rad(180)] = 0
Cs3[theta<np.deg2rad(120)] = 0
Cs3[theta>np.deg2rad(180)] = 0

z0 = (1 + X + Z)/2
As4 = z0 - Z
Bs4 = z0 
Cs4 = z0 - X

As4[theta<np.deg2rad(180)] = 0
As4[theta>np.deg2rad(240)] = 0
Bs4[theta<np.deg2rad(180)] = 0
Bs4[theta>np.deg2rad(240)] = 0
Cs4[theta<np.deg2rad(180)] = 0
Cs4[theta>np.deg2rad(240)] = 0

z0 = (1 + Y - Z)/2
As5 = z0
Bs5 = z0 + Z 
Cs5 = z0 - Y

As5[theta<np.deg2rad(240)] = 0
As5[theta>np.deg2rad(300)] = 0
Bs5[theta<np.deg2rad(240)] = 0
Bs5[theta>np.deg2rad(300)] = 0
Cs5[theta<np.deg2rad(240)] = 0
Cs5[theta>np.deg2rad(300)] = 0

z0 = (1 - X - Y)/2
As6 = z0 + Y
Bs6 = z0 + X 
Cs6 = z0

As6[theta<np.deg2rad(300)] = 0
Bs6[theta<np.deg2rad(300)] = 0
Cs6[theta<np.deg2rad(300)] = 0

A = As1 + As2 + As3 + As4 + As5 + As6
B = Bs1 + Bs2 + Bs3 + Bs4 + Bs5 + Bs6
C = Cs1 + Cs2 + Cs3 + Cs4 + Cs5 + Cs6

#pltXYZ = plt.subplot(211)
plt.plot(theta, X, label = r'X: $\beta$', linestyle='--')
plt.plot(theta, Y, label = r'Y: $\frac{1}{2}(\beta + \sqrt{3}\alpha)$', linestyle='--')
plt.plot(theta, Z, label = r'Z: $\frac{1}{2}(\beta - \sqrt{3}\alpha)$', linestyle='--')
#plt.plot(theta, -X, label = r'-X', linestyle='--')
#plt.plot(theta, -Y, label = r'-Y', linestyle='--')
#plt.plot(theta, -Z, label = r'-Z', linestyle='--')

#pltABC = plt.subplot(212)
plt.plot(theta, A, label = 'A')
plt.plot(theta, B, label = 'B')
plt.plot(theta, C, label = 'C')

plt.xlim(0, np.deg2rad(360))
plt.xticks(angles, anglelabels)

plt.xlabel('Angle')
plt.ylabel('Amplitude')
plt.title('Vector Magnitude / SVPWM')
plt.grid(True)
plt.legend(loc='lower right', fancybox=True, shadow=True)

sectorcenters = [np.deg2rad(30), np.deg2rad(90), np.deg2rad(150), np.deg2rad(210), np.deg2rad(270), np.deg2rad(330)]
sectorlabels = ['Sector 1', 'Sector 2', 'Sector 3', 'Sector 4', 'Sector 5', 'Sector 6']
sectoraxes = plt.twiny()
sectoraxes.set_xticks(sectorcenters)
sectoraxes.set_xlim(0, np.deg2rad(360))
sectoraxes.set_xticklabels(sectorlabels)
sectoraxes.tick_params('x', length = 0)
#sectoraxes.xaxis.set_ticks_position('bottom')

plt.show()
