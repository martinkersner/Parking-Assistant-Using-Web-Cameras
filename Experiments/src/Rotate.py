#!/usr/bin/python

# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Rotate to disparity map about 90 degrees to see top view.
#
# m.kersner@gmail.com
# 01/22/2015

import numpy as np

class Rotate:

    img        = None
    cols       = None
    img_rotate = None

    def __init__(self, img, layers=256):
        self.img = img
        self.cols = img.shape[1]
        self.img_rotate = np.zeros((layers, img.shape[1]))

    def rotate90(self):
        for c in range(0, self.img.shape[1]):
            depth_set = self.create_set(self.img[:, c])

            for s in depth_set:
                self.img_rotate[s, c] = s

        return self.normalize2(self.img_rotate)

    def create_set(self, col):
        return set(col)

    def normalize2(self, X):
        x_min = np.amin(X)
        x_max = np.amax(X)

        return (((1.0*X) - x_min)) / (x_max - x_min)
