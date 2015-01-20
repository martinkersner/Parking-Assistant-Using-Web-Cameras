#!/usr/bin/python
# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Create PLY file out of image.
#
# m.kersner@gmail.com
# 01/19/2015

import cv2
import numpy as np
import os.path
import sys

def CreateHeader(number_vertex, rows=None, cols=None, t='triangular'):
    print "ply"
    print "format ascii 1.0"
    print "element vertex ", number_vertex
    print "property float x"
    print "property float y"
    print "property float z"
    print "property uchar diffuse_red"
    print "property uchar diffuse_green"
    print "property uchar diffuse_blue"

    if (rows != None and cols != None):
        if (t == 'square'):
            print "element face", number_vertex-rows-cols
        elif (t == 'triangular'):
            print "element face", (number_vertex-rows-cols) * 2

        print "property list uchar int vertex_indices"

    print "end_header"

def ReadImage(image_path):
    img = None
    supported_imgs = ( '.png',
                       '.jpg',
                       '.jpeg' )

    if (image_path.lower().endswith(supported_imgs) and os.path.isfile(image_path)):
        img = cv2.imread(image_path, 0)

    return img

def CreateFaces(number_vertex, rows, cols, t='triangular'):

    for i in range(0, number_vertex):
        if (i >= number_vertex-cols-1):
            break

        # square face
        if (t == 'square'):
            if ((i+1) % cols != 0):
                print 4, i, i+1, i+cols, i+cols+1

        # triangular faces
        elif (t == 'triangular'):
            if ((i+1) % cols != 0):
                print 3, i, i+cols, i+cols+1
                print 3, i, i+1, i+cols+1

def CreatePalette(palette_range=256):
    ''' Creates pallete consisted of colors which can be used to colorize 3D
        model.
        TODO find a better way
    '''
    palette = list()

    R = 0
    G = 255
    B = 255

    step = (256+255+255)/palette_range

    for b in range(0, 256, step):
        palette.append((R, G, B-b))

    B = 0

    for r in range(0, 256, step):
        palette.append((R+b, G, B))

    R = 255

    for g in range(0, 256, step):
        palette.append((R, G-b, B))

    return palette

def main():
    if (len(sys.argv) > 1):
        img = ReadImage(sys.argv[1])

        if (img == None):
            sys.stderr.write("Given path to file does not exist, or image file is not supported!\n")
            exit(-1)

    else:
        sys.stderr.write("Script has to be run with argument as path to examined image!\n")
        exit(-1)

    p = CreatePalette()

    CreateHeader(img.size, img.shape[0], img.shape[1])

    for i in range(0, img.shape[0]):
        for j in range(0, img.shape[1]):
            v = img[i,j]
            print i, j, img[i,j], p[v][0], p[v][1], p[v][2]

    CreateFaces(img.size, img.shape[0], img.shape[1])

if __name__ == '__main__': main()
