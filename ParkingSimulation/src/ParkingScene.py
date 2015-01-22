# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Reader of Parking Scene
#
# ps = ParkingScene('name_of_file')
# params = ps.get_params()
#
# m.kersner@gmail.com
# 01/22/2015

import os.path
import numpy as np

class ParkingScene():

    params = None
    f      = None
    H      = None
    I      = None
    J      = None
    K      = None
    P      = None
    A      = None

    def __init__(self, file_name):
        if (os.path.isfile(file_name)):
            self.f = open(file_name, "r")
            self.read()
            self.f.close()
        else:
            sys.stderr.write("Given file does not exist!\n")
            exit(-1)

    def read(self):
        # parking lot
        self.H = self.get_point()
        self.I = self.get_point()
        self.J = self.get_point()
        self.K = self.get_point()

        # car
        self.P = self.get_point()
        self.A = self.get_number()

    def get_params(self):
        self.params = dict( H = self.H,
                            I = self.I,
                            J = self.J,
                            K = self.K,
                            P = self.P,
                            A = self.A )

        return self.params

    def get_point(self):
        for line in self.f:
            if (not self.is_empty(line)):
                return self.create_point(line)

    def get_number(self):
        for line in self.f:
            if (not self.is_empty(line)):
                return int(line)

    def create_point(self, string):
        spl = string.split()

        X = int(spl[0])
        Y = int(spl[1])

        return np.array((X, Y))

    def is_empty(self, line):
        if (line[0] != "#" and line.strip()):
            return False
        else:
            return True
