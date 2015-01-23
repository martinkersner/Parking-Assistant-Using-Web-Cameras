#!/usr/bin/python

# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Quality Metrics for purpose of comparing computed disparity map and ground
# truth disparity map.
# Expects greyscale images with range 0 to 255.
#
# RMSE  Root Mean Square Error
# BM    Percentage of Bad Matching Pixels
#
# m.kersner@gmail.com
# 01/23/2015

import numpy as np

class QualityMetrics:
    '''
    Disparity map and ground truth disparity map are interchangeable due to
    power of 2 in RMSE and absolute value in BM.
    '''

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

    def bm(self, threshold=1.0):
        '''
        Percentage of Bad Matching Pixels
        '''
        noe = 1.0*self.dc.size # number of elements
        mask = np.fabs(self.dc-self.dt) > threshold

        return np.sum(mask.reshape((1, noe))) / noe

