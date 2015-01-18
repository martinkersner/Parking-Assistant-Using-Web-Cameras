# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Creating Stereo Image
#
# m.kersner@gmail.com
# 01/10/2015

import numpy as np
import cv2

class Stereo:
    ## STEREO BLOCK MATCHING ###################################################
    stereoBM_params = dict(# Specifies the whole set of algorithm parameters, one of:
                           # BASIC_PRESET    - parameters suitable for general cameras
                           # FISH_EYE_PRESET - parameters suitable for wide-angle cameras
                           # NARROW_PRESET   - parameters suitable for narrow-angle cameras
                           preset=0,

                           # The disparity search range. For each pixel algorithm
                           # will find the best disparity from 0 (default minimum
                           # disparity) to ndisparities. The search range can then
                           # be shifted by changing the minimum disparity.
                           # Must be divisible by 16!
                           ndisparities=48,

                           # The linear size of the blocks compared by the algorithm.
                           # The size should be odd (as the block is centered at the
                           # current pixel). Larger block size implies smoother, though
                           # less accurate disparity map. Smaller block size gives more
                           # detailed disparity map, but there is higher chance for
                           # algorithm to find a wrong correspondence.
                           SADWindowSize=11)

    ## SEMI-GLOBAL BLOCK MATCHING ##############################################
    stereSGMB_params = dict(# Minimum possible disparity value. Normally, it is
                            # zero but sometimes rectification algorithms can
                            # shift images, so this parameter needs to be
                            # adjusted accordingly.
                            minDisparity=-64,

                            # Maximum disparity minus minimum disparity.
                            # The value is always greater than zero. In the current
                            # implementation, this parameter must be divisible by 16.
                            numDisparities=192,

                            # Matched block size. It must be an odd number >=1 .
                            # Normally, it should be somewhere in the 3..11 range.
                            SADWindowSize=5,

                            # The first parameter controlling the disparity smoothness.
                            P1=600,

                            # The second parameter controlling the disparity
                            # smoothness. The larger the values are, the smoother
                            # the disparity is. P1 is the penalty on the disparity
                            # change by plus or minus 1 between neighbor pixels.
                            # P2 is the penalty on the disparity change by more
                            # than 1 between neighbor pixels.
                            # The algorithm requires P2 > P1
                            P2=2400,

                            # Maximum allowed difference (in integer pixel units)
                            # in the left-right disparity check.
                            # Set it to a non-positive value to disable the check.
                            disp12MaxDiff=10,

                            # Truncation value for the prefiltered image pixels.
                            # The algorithm first computes x-derivative at each pixel
                            # and clips its value by [-preFilterCap, preFilterCap]
                            # interval. The result values are passed to
                            # the Birchfield-Tomasi pixel cost function.
                            preFilterCap=4,

                            # Margin in percentage by which the best (minimum)
                            # computed cost function value should "win"
                            # the second best value to consider the found
                            # match correct. Normally, a value within
                            # the 5-15 range is good enough.
                            uniquenessRatio=1,

                            # Maximum size of smooth disparity regions to consider
                            # their noise speckles and invalidate. Set it to 0
                            # to disable speckle filtering.
                            # Otherwise, set it somewhere in the 50-200 range.
                            speckleWindowSize=150,

                            # Maximum disparity variation within each connected
                            # component. If you do speckle filtering,
                            # set the parameter to a positive value, it will
                            # be implicitly multiplied by 16.
                            # Normally, 1 or 2 is good enough.
                            speckleRange=2,

                            # Set it to true to run the full-scale two-pass
                            # dynamic programming algorithm. It will consume
                            # O(W*H*numDisparities) bytes, which is large
                            # for 640x480 stereo and huge for HD-size pictures.
                            # By default, it is set to false .
                            fullDP=False )

    ## SPECKLES ################################################################
    speckles_params = dict( # The disparity value used to paint-off the speckles.
                            newVal=0,

                            # The maximum speckle size to consider it a speckle.
                            maxSpeckleSize=200,

                            # Larger blobs are not affected by the algorithm.
                            # Maximum difference between neighbor disparity pixels
                            # to put them into the same blob. Note that since StereoBM,
                            # StereoSGBM and may be other algorithms return a
                            # fixed-point disparity map, where disparity values are
                            # multiplied by 16, this scale factor should be taken
                            # into account when specifying this parameter value.
                            maxDiff=48 )

    disparity   = None
    disparity90 = None
    imgL        = None
    imgR        = None
    imgL90      = None
    imgR90      = None
    stereo      = None

    def __init__(self, img_left=None, img_right=None):
        if (img_left != None and img_right != None):
            self.load_images(img_left, img_right)


    def load_images(self, img_left, img_right):
        self.imgL = cv2.imread(img_left, 0)
        self.imgR = cv2.imread(img_right, 0)

        # rotate images
        self.imgL90 = np.flipud(np.transpose(self.imgL))
        self.imgR90 = np.flipud(np.transpose(self.imgR))

    def BM(self):
        self.stereo = cv2.StereoBM(**self.stereoBM_params)

    def SGBM(self):
        self.stereo = cv2.StereoSGBM(**self.stereSGMB_params)

    def compute_disparity(self):
        self.disparity = self.stereo.compute(self.imgL, self.imgR)
        self.disparity90 = self.stereo.compute(self.imgL90, self.imgR90)

    def filter_speckles(self):
        cv2.filterSpeckles(self.disparity, **self.speckles_params)
        cv2.filterSpeckles(self.disparity90, **self.speckles_params)

    def display_disparity(self, equalize=False):
        disparity, disparity_all = self.remap_intensities(equalize)

        cv2.imshow('Disparity', disparity)
        cv2.imshow('DisparityAll', disparity_all)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def get_disparity(self, equalize=False):
        return self.remap_intensities(equalize)

    def remap_intensities(self, equalize=False):
        if (equalize == True):
            disparity     = cv2.equalizeHist(np.asmatrix(self.disparity, dtype='uint8'))
            disparity_all = cv2.equalizeHist(np.asmatrix(self.disparity +
                              np.transpose(np.flipud(self.disparity90)), dtype='uint8'))

        else:
            disparity     = self.normalize2(self.disparity)
            disparity_all = self.normalize2(self.disparity +
                              np.transpose(np.flipud(self.disparity90)))

        return disparity, disparity_all

    def normalize2(self, X):
        x_min = np.amin(X)
        x_max = np.amax(X)

        return (((1.0*X) - x_min)) / (x_max - x_min)
