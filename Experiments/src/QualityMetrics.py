#!/usr/bin/python

# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Quality Metrics for purpose of comparing computed disparity map and ground
# truth disparity map.
#
# Root Mean Square Error
# Percentage of Bad Matching Pixels
#
# m.kersner@gmail.com
# 01/23/2015

import numpy as np

class QualityMetrics:

    dc = None  # disparity map
    dt = None  # groudn truth disparity map

    def __init__(self, dc, dt):
        if (self.control_size(dc, dt)):
            self.dc = dc
            self.dt = dt

    def control_size(self, img1, img2):
        if (img1.shape == img2.shape):
            return True
        else:
            return False

    def rmse(self):
        '''
        Root Mean Square Error
        '''
        noe = self.dc.size # number of elements
        return np.sqrt(np.sum(np.power(self.dc-self.dt, 2).reshape((1, noe))) / noe)
