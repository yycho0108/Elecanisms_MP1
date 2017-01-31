#!/usr/bin/python
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np

sectorSize = 100

def plotcsv(name):
    data = np.loadtxt(name)
    sector = len(data)/sectorSize
    df = pd.DataFrame(data, columns=['Encoder', 'Camera', 'Bias'])
    print len(df)
    df = df[df.Camera != -1]
    print len(df)
    df.plot()
    plt.title('Vision-Based Encoder Calibration')
    plt.show()

plotcsv('calib.csv')
