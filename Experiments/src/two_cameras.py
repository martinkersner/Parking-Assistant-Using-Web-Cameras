# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Capturing Two Cameras
#
# m.kersner@gmail.com
# 01/10/2015

import numpy as np
import cv2

def normalize2(X):
    ''' Normalize 2D array
    '''
    x_min = np.amin(X)
    x_max = np.amax(X)

    return (((1.0*disparity) - x_min)) / (x_max - x_min)

def main():
    cap_left  = cv2.VideoCapture(1)
    cap_right = cv2.VideoCapture(2)

    print 'LEFT: ', cap_left.get(3), cap_left.get(4)
    print 'RIGHT: ', cap_right.get(3), cap_right.get(4)

    index = 0
    stereo = cv2.StereoBM(0, 64, 41)
    stereo_vision = False

    while(True):
        ret_left, frame_left   = cap_left.read()
        ret_right, frame_right = cap_right.read()

        k = cv2.waitKey(30)

        if (k == ord('q')):
            break
         # capture
        elif (k == ord('c')):
            cv2.imwrite('../img/left.png', frame_left)
            cv2.imwrite('../img/right.png', frame_right)
        # capture right
        elif (k == ord('r')):
            cv2.imwrite('../img/right' + str(index) + '.png', frame_right)
            index+=1
        # stereo vision mode
        elif (k == ord('s')):
            stereo_vision = True

        if (stereo_vision):
            gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY);
            gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY);

            disparity = stereo.compute(gray_left, gray_right)

            cv2.imshow('stereo', normalize2(disparity))
            cv2.imshow('frame_left', frame_left)
            cv2.imshow('frame_right', frame_right)
        else:
            cv2.imshow('frame_left', frame_left)
            cv2.imshow('frame_right', frame_right)

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

if __name__ == '__main__': main()
