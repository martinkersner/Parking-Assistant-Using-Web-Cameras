# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Capturing Two Cameras
#
# m.kersner@gmail.com
# 01/10/2015

import numpy as np
import cv2

cap_left  = cv2.VideoCapture(1)
cap_right = cv2.VideoCapture(2)

print 'LEFT: ', cap_left.get(3), cap_left.get(4)
print 'RIGHT: ', cap_right.get(3), cap_right.get(4)

index = 0

while(True):
    ret_left, frame_left   = cap_left.read()
    ret_right, frame_right = cap_right.read()

    cv2.imshow('frame_left', frame_left)
    cv2.imshow('frame_right', frame_right)

    k = cv2.waitKey(30)

    if (k == ord('q')):
        break
     # capture
    elif (k == ord('c')):
        cv2.imwrite('../images/left.png', frame_left)
        cv2.imwrite('../images/right.png', frame_right)
    # capture right
    elif (k == ord('r')):
        cv2.imwrite('../images/right' + str(index) + '.png', frame_right)
        index+=1

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
