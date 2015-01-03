#!/usr/bin/python
#
# Parking Assistant Using Web Cameras
# Martin Kersner's Master Thesis
#
# Parking Simulation
#
# m.kersner@gmail.com
# 01/03/2015

import sys
import math
import os
import pygame
from pygame.locals import *

def LoadPng(name):
    image = pygame.image.load(name)
    if image.get_alpha is None:
        image = image.convert()
    else:
        image = image.convert_alpha()

    return image, image.get_rect()

class Car(pygame.sprite.Sprite):

    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.originalImage, self.originalRect = LoadPng('../img/car.png')
        self.image = self.originalImage
        self.rect = self.originalRect
        screen = pygame.display.get_surface()
        self.area = screen.get_rect()

        self.speed = 5
        self.steeringSpeed = 3
        self.steering = 2

        self.angle = 0
        self.x = 100
        self.y = 100

        self.Reinit()

    def Reinit(self):
        self.movePosition = [0,0]
        self.rect.x = self.x
        self.rect.y = self.y
        self.image = pygame.transform.rotozoom(self.image, self.angle, 1)

    def update(self):
        newPosition = self.rect.move(self.movePosition)
        if self.area.contains(newPosition):
            self.rect = newPosition

        pygame.event.pump()

    def NextPosition(self, speed):
        x = speed * math.sin(math.radians(self.angle))
        y = speed * math.cos(math.radians(self.angle))

        return x, y

    def NewPosition(self, x, y, direction):
        if (self.angle > 0 and self.angle < 90):
            self.movePosition[0] -= x
            self.movePosition[1] -= y
        elif (self.angle > 90 and self.angle < 180):
            self.movePosition[0] -= x
            self.movePosition[1] += y
        elif (self.angle > 180 and self.angle < 270):
            self.movePosition[0] += x
            self.movePosition[1] += y
        elif (self.angle > 270 and self.angle < 360):
            self.movePosition[0] += x
            self.movePosition[1] -= y

        if (direction == 'backward'):
            self.movePosition[0] *= -1
            self.movePosition[1] *= -1

    def MoveForward(self):
        x, y = self.NextPosition(self.speed)

        self.movePosition[0] -= x;
        self.movePosition[1] -= y;

    def MoveBackward(self):
        x, y = self.NextPosition(self.speed)

        self.movePosition[0] += x
        self.movePosition[1] += y

    def Turn(self, steering, direction):
        self.angle = (self.angle + steering) % 360

        x, y = self.NextPosition(self.steeringSpeed)

        # TODO leaves small parts of rotated image behind its path
        self.image, self.rect = self.RotationCenter(self.originalImage, self.rect, self.angle)
        #self.image = pygame.transform.rotozoom(self.originalImage, self.angle, 1)

        self.NewPosition(x, y, direction)

    def RotationExperiment(self):
        self.angle += 30
        #self.image = pygame.transform.rotate(self.image1, self.angle+30)
        self.image, self.rect = self.RotationCenter(self.originalImage, self.rect, self.angle)

    def RotationCenter(self, image, rect, angle):
        """rotate an image while keeping its center"""
        rotImage = pygame.transform.rotozoom(image, angle, 1)
        #rotImage = pygame.transform.rotate(image, angle)
        rotRect = rotImage.get_rect(center=rect.center)
        return rotImage,rotRect

def main():
    # Initialize screen
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('Parking Simulation')

    # Fill background
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((255, 255, 255))

    car = Car()
    carSprites = pygame.sprite.RenderPlain(car)

    # Blit everything to the screen
    screen.blit(background, (0, 0))
    pygame.display.flip()

	# Initialize clock
    clock = pygame.time.Clock()

    while 1:
        clock.tick(60)

        keys = pygame.key.get_pressed()
        if keys[K_LEFT] and keys[K_UP]:
            car.Turn(car.steering, 'forward')
            car.movePosition = [0,0]
        if keys[K_RIGHT] and keys[K_UP]:
            car.Turn(-car.steering, 'forward')
            car.movePosition = [0,0]
        if keys[K_LEFT] and keys[K_DOWN]:
            car.Turn(-car.steering, 'backward')
            car.movePosition = [0,0]
        if keys[K_RIGHT] and keys[K_DOWN]:
            car.Turn(car.steering, 'backward')
            car.movePosition = [0,0]
        if keys[K_UP]:
            car.MoveForward()
        if keys[K_DOWN]:
            car.MoveBackward()

        # TODO experiment
        if keys[K_TAB]:
            car.RotationExperiment()

        for event in pygame.event.get():
            if event.type == QUIT:
                return

        screen.blit(background, car.rect)
        carSprites.update()
        carSprites.draw(screen)
        pygame.display.flip()
        car.movePosition = [0,0]

if __name__ == '__main__': main()
