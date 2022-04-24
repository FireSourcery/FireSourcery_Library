import matplotlib.pyplot as plt
import numpy as np

def fix16_div(a, b): return (a*65536)/b/2
def fix16_mul(a, b): return (a*b)/(65536/2)

def angle16(deg): return np.int16(deg*65536/360)

def atan2_f(y, x):
    #mask = (y >> (4*8-1));
    #abs_inY = (y + mask) ^ mask;
    r = x * 0
    r_3 = x * 0
    angle = x * 0
    yAbs = np.abs(y)
    for i in range(len(x)):
        if(x[i] >= 0):
            r[i] = fix16_div((x[i] - yAbs[i]), (x[i] + yAbs[i]))
            r_3[i] = fix16_mul(fix16_mul(r[i], r[i]), r[i])
            angle[i] = fix16_mul(0x00003240/2, r_3[i]) - fix16_mul(0x0000FB50/2, r[i]) + 0x0000C90F/2 #PI_DIV_4
        else:
            r[i] = fix16_div((x[i] + yAbs[i]), (yAbs[i] - x[i]))
            r_3[i] = fix16_mul(fix16_mul(r[i], r[i]), r[i])
            angle[i] = fix16_mul(0x00003240/2, r_3[i]) - fix16_mul(0x0000FB50/2, r[i]) + 0x00025B2F/2 #3PI_DIV_4
        if(y[i] < 0):
            angle[i] = -angle[i]
    return angle 

theta = np.linspace(0, np.deg2rad(360), 500)

cosine = np.cos(theta) * 32767 
sine = np.sin(theta) * 32767 
atan2 = atan2_f(sine, cosine) / np.pi

atan2_4 = np.int16(atan2_f(sine, cosine)/4 / np.pi)

#pltXYZ = plt.subplot(211)
plt.plot(theta, sine, label = r'Sine', linestyle='--')
plt.plot(theta, cosine, label = r'Cosine', linestyle='--') 

#pltABC = plt.subplot(212)
plt.plot(theta, atan2, label = 'Atan2')  
plt.plot(theta, atan2_4, label = 'Atan2 4x')  

angles = [np.deg2rad(0), np.deg2rad(60), np.deg2rad(120), np.deg2rad(180), np.deg2rad(240), np.deg2rad(300), np.deg2rad(360)]
#anglelabels = ['60', '120', '180', '240', '300', '360']
anglelabels = [angle16(0), angle16(60), angle16(120), angle16(180), angle16(240), angle16(300), angle16(360)]

plt.xlim(0, np.deg2rad(360))
plt.xticks(angles, anglelabels)

plt.xlabel('Angle')
plt.ylabel('Frac16')
plt.title('Atan2')
plt.grid(True)
plt.legend(loc='lower right', fancybox=True, shadow=True)

##sectorcenters = [np.deg2rad(30), np.deg2rad(90), np.deg2rad(150), np.deg2rad(210), np.deg2rad(270), np.deg2rad(330)]
##sectorlabels = ['Sector 1', 'Sector 2', 'Sector 3', 'Sector 4', 'Sector 5', 'Sector 6']

sectorcenters = [np.deg2rad(60), np.deg2rad(120), np.deg2rad(180), np.deg2rad(240), np.deg2rad(300), np.deg2rad(360)]
sectorlabels = ['60', '120', '180', '240', '300', '360']

sectoraxes = plt.twiny()
sectoraxes.set_xticks(sectorcenters)
sectoraxes.set_xlim(0, np.deg2rad(360))
sectoraxes.set_xticklabels(sectorlabels)
##sectoraxes.tick_params('x', length = 0)
#sectoraxes.xaxis.set_ticks_position('bottom')

plt.show()
