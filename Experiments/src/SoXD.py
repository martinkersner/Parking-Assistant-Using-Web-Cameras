#!/usr/bin/python

# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Sum of Absolute/Squared Differences
#
# m.kersner@gmail.com
# 01/18/2015

import numpy as np
from scipy import signal
import cv2

class SoXD:
    minSSD      = None
    disparity   = None
    imgL        = None
    imgR        = None
    max_shift   = None
    window_size = None
    window      = None
    sum_type    = None

    def __init__(self, img_left, img_right, sum_type='absolute', max_shift=10, window_size=7):
        self.load_images(img_left, img_right)
        self.max_shift = max_shift
        self.window_size = window_size
        self.window = np.ones((self.window_size, self.window_size))
        self.sum_type = sum_type

    def load_images(self, img_left, img_right):
        self.imgL = cv2.imread(img_left, 0)
        self.imgR = cv2.imread(img_right, 0)

    def pad_image(self, img):
        ''' Add padding (zeros) at image borders '''
        return np.pad(img, ((1,1),(1,1)), mode='constant')

    def shift_image(self, img, k):
        ''' Shift the image to the right by k pixels '''
        if (k != 0):
            return np.pad(img, ((0,0),(k,0)), mode='constant')[:, :-k]
        else:
            return img

    def ssd(self, imgL, imgR):
        squared_diff = (imgL-imgR)**2
        return signal.convolve2d(squared_diff, self.window, mode='same')

    def sad(self, imgL, imgR):
        squared_diff = np.abs(imgL-imgR)
        return signal.convolve2d(squared_diff, self.window, mode='same')

    def get_disparity(self):
        #  TODO use the maximum value of integer
        default_intensity = 100000

        disparity = np.zeros_like(self.imgL)
        min_sxd   = np.ones_like(self.imgL) * default_intensity

        for k in range(0, self.max_shift):
            shift_imgR = self.shift_image(self.imgR, k)

            if (self.sum_type == 'absolute'):
                sxd = self.sad(self.imgL, shift_imgR)
            elif (self.sum_type == 'square'):
                sxd = self.ssd(self.imgL, shift_imgR)

            sxd_mask = sxd < min_sxd
            sxd_mask2 = sxd > min_sxd

            min_sxd = np.multiply(min_sxd, sxd_mask2) + np.multiply(sxd, sxd_mask)
            disparity = np.multiply(disparity, sxd_mask2) + np.multiply(np.ones_like(self.imgL), sxd_mask) * k

        # removing outliers
        disparity = cv2.medianBlur(disparity, 11)
        return self.normalize2(disparity)

    def normalize2(self, X):
        x_min = np.amin(X)
        x_max = np.amax(X)

        return (((1.0*X) - x_min)) / (x_max - x_min)
