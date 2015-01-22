#!/usr/bin/python
#
# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Motion simulation
#
# m.kersner@gmail.com
# 01/22/2015

import math
import numpy as np
import sys

class Motion:

    # VEHICLE PROPERTIES
    P     = np.array((0,0))  # vehicle position X
    L     = None  # wheelbase
    Vp    = None  # current vehicle velocity
    phi   = None  # angle of wheels
    theta = None  # heading of vehicle

    Vc    = None  # closing velocity
    Vt    = np.array((0,0))
    nc    = None  # acceleration closing
    nt    = 0
    Vtp   = None

    lmbd   = None
    lmbd_f = None
    he     = None  # heading error

    R     = np.array((0,0)) # range vector
    t_go  = None  # time to go

    def __init__(self):

    def compute_heading(self):
        '''
        Compute heading (angle) of vehicle.
        Use degrees for computation.
        '''
        return (self.Vp / self.L) * math.sin(math.radians(self.phi))

    def control_steering_angle(self):
        '''
        Use degrees for control.
        '''
        if (self.phi > 90):
            return False
        else:
            return True

    def compute_Vtp(self):
        self.Vtp = self.Vt - self.Vp

    def compute_Vc(self):
        '''
        Compute closing velocity.
        '''
        self.compute_R() #TODO is it necessary?
        self.Vc = -math.fabs(R[0]*Vtp[1] - R[1]*Vtp[2])/self.vector_length(self.R)

    def vector_length(self, vec):
        if len(vec) == 2):
            return mat.sqrt(vec[0]**2 + vec[1]**2)
        else:
            sys.stdexrr.write("Vector has to consist exactly from two components!\n")
            exit(-1)

    def compute_R(self):
        self.R[0] = math.fabs(self.T[0] - self.P[0])
        self.R[1] = math.fabs(self.T[1] - self.P[1])

    def compute_t_go(self):
        self.compute_R() #TODO is it necessary?
        self.t_go = self.vector_length(self.R) / self.vector_length(self.Vc)

    def compute_heading_eror(self):
        '''
        Use degrees.
        '''
        self.he = self.theta - self.lmbd

    # TODO add lambda with dot above
    def compute_nc(self):
        diff_lmbd = math.fabs(self.lmbd - self.lmbd_f)
        self.nc[0] = 4*self.Vc[0] + (2*self.Vc[0]*diff_lmbd)/(self.t_go) + self.nt
        self.nc[1] = 4*self.Vc[1] + (2*self.Vc[1]*diff_lmbd)/(self.t_go) + self.nt
