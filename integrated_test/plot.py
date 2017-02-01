#!/usr/bin/python
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np

from cmath import rect,phase
from math import radians, degrees

def mean_angle(deg):
    return degrees(phase(sum(rect(1, radians(d)) for d in deg)/len(deg))) 

sectorSize = 100

def plotcsv(name):
    data = np.loadtxt(name)
    data[:,2] = np.unwrap(data[:,2])
    plt.plot(data[:,2])

    sector = len(data)/sectorSize
    df = pd.DataFrame(data, columns=['Encoder', 'Camera', 'Bias'])
    #df.Encoder, df.Camera, df.Bias = [np.unwrap(np.deg2rad(x)) for x in (df.Encoder,df.Camera,df.Bias)]
    df = df[df.Camera != -1]

    df['Bias'] = np.rad2deg(np.unwrap(np.deg2rad(df['Bias'])))

    df.plot()
    plt.title('Vision-Based Encoder Calibration')

    print 'median offset', np.rad2deg(np.median(df.Bias))

    plt.show()

plotcsv('calib.csv')
