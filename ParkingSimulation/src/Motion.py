#!/usr/bin/python
#
# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Motion simulation
# Guidance Based Autonomous Parking Assistant
#
# m.kersner@gmail.com
# 01/22/2015

import math
import numpy as np
import sys

# TODO
# 1/ angle correction

class Motion:

    # VEHICLE PROPERTIES
    P     = np.array((0,0))  # vehicle position
    Pp    = np.array((0,0))  # previous vehicle position
    L     = None  # wheelbase
    Vp    = None  # current vehicle velocity
                  # Vp is eaqual to d
    phi   = None  # angle of wheels
    theta = None  # heading of vehicle

    Vc    = None  # closing velocity
    Vt    = np.array((0,0))
    nc    = None  # acceleration closing
    nt    = 0
    Vtp   = None

    lmbd   = None
    lmbd_f = None
    lmbd_f_first = None
    lmbd_f_second = None
    he     = None  # heading error

    t_go  = None  # time to go

    R     = np.array((0,0))  # range vector
    Z     = np.array((0,0))  # point in desired direction of approach
    D     = np.array((0,0))  # direction vector

    # PARKING LOT
    #     K --------------- J
    #                       |
    # C   B        A        |
    #                       |
    #     H --------------- I
    H = np.array((0,0))  # opening of parking space
    I = np.array((0,0))
    J = np.array((0,0))
    K = np.array((0,0))  # opening of parking space

    # auxiliary points for computation of desired trajectory
    A = np.array((0,0))
    B = np.array((0,0))
    C = np.array((0,0))
    epsilon = None  # safety distance

    def __init__(self, **params):
        a = 0

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
        if (len(vec) == 2):
            return mat.sqrt(vec[0]**2 + vec[1]**2)
        else:
            sys.stdexrr.write("Vector has to consist exactly from two components!\n")
            exit(-1)

    def compute_R(self):
        '''
        Compute range vector.
        '''
        self.R[0] = math.fabs(self.T[0] - self.P[0])
        self.R[1] = math.fabs(self.T[1] - self.P[1])

    def compute_D(self):
        '''
        Compute direction vector.
        '''
        self.D[0] = math.fabs(self.Z[0] - self.T[0])
        self.D[1] = math.fabs(self.Z[1] - self.T[1])

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
        '''
        Compute acceleration command.
        '''
        diff_lmbd = math.fabs(self.lmbd - self.lmbd_f)
        self.nc[0] = 4*self.Vc[0] + (2*self.Vc[0]*diff_lmbd)/(self.t_go) + self.nt
        self.nc[1] = 4*self.Vc[1] + (2*self.Vc[1]*diff_lmbd)/(self.t_go) + self.nt

    # TODO control order of X and Y
    def compute_lmbd(self):
        '''
        Use degrees.
        '''
        self.lmbd = math.degrees(math.atan( self.R[1] / self.R[0] ))

    # TODO control order of X and Y
    def compute_lmbd_f(self):
        '''
        Use degrees.
        '''
        self.lmbd_f = math.degrees(math.atan( self.D[1] / self.D[0] ))

    def control_nonholonomic_theta():
        d = self.Vp

        if (self.theta > (d / self.L)):
            return False
        else:
            return True

    def compute_new_position(self):
        alpha = self.theta
        d = self.Vp
        tmp_position = self.P

        cos_alpha = math.cos(math.radians(alpha))
        sin_alpha = math.sin(math.radians(alpha))

        self.P = self.P + d*np.array((cos_alpha, sin_alpha))
        self.Pp = tmp_position

    def compute_abc(self):
        self.A = (H + I + J + K) / 4.0
        self.B = (H + K) / 2.0

        IH = self.create_vector(self.I, self.H)
        unit_IH = self.unit_vector(IH)

        self.C = self.B + epsilon * unit_IH

    def unit_vector(self, vec):
        '''
        Expect vector with 2 dimensions.
        '''
        m = math.sqrt(vec[0]**2 + vec[1]**2)
        return np.array((vec[0]/m, vec[1]/m))

    def create_vector(self, A, B):
        '''
        Creates vector from two given points.
        Point A is beginning and point is end of vector.
        '''
        return np.array((B.shape[0]-A.shape[0], B.shape[1]-A.shape[1]))

    def case1():
        '''
        1/ Vehicle goes to C with lambda_f_first
        2/ Vehicle goes to A with lambda_f_first
        '''
        HI = self.create_vector(self.H, self.I)
        self.lmbd_f_first = math.degrees(math.atan( self.HI[1] / self.HI[0] ))

    # TODO implement
    def case2():
        a = 0

    def park(self):
        compute_heading_eror()

        if (self.he <= 70):
            self.case1()
        else:
            print "not implemented yet"
            self.case2()
