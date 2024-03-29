# -*- coding: utf-8 -*-
from progeo import *
import cv2
import pygame
import timeit
import time


# defino variables globales:
ANCHO_IMAGEN = 640
LARGO_IMAGEN = 480

class Camera:
    def __init__(self, k11, k12, k22, k23, k13, rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33, position):
        self.k11 = k11
        self.k12 = k12
        self.k22 = k22
        self.k23 = k23
        self.k13 = k13
        self.rt11 = rt11
        self.rt12 = rt12
        self.rt13 = rt13
        self.rt21 = rt21
        self.rt22 = rt22
        self.rt23 = rt23
        self.rt31 = rt31
        self.rt32 = rt32
        self.rt33 = rt33
        self.position = position

class Punto2D:
    def __init__(self, x, y, h):
        self.x = x
        self.y = y
        self.h = h

class Punto3D:
    def __init__(self, x, y, z, h):
        self.x = x
        self.y = y
        self.z = z
        self.h = h

def backproject (punto2D, camera):
	output = -1
	temp2D = Punto2D()
	punto3D = Punto3D()

	if (punto2D.h>0.):
		temp2D.h=camera.k11
		temp2D.x=punto2D.x*camera.k11/punto2D.h
		temp2D.y=punto2D.y*camera.k11/punto2D.h

		ik11=(1./camera.k11)
		ik12=-camera.k12/(camera.k11*camera.k22)
		ik13=(camera.k12*camera.k23-camera.k13*camera.k22)/(camera.k22*camera.k11)
		ik21=0.
		ik22=(1./camera.k22)
		ik23=-(camera.k23/camera.k22)
		ik31=0.
		ik32=0.
		ik33=1.

		a1=ik11*temp2D.x+ik12*temp2D.y+ik13*temp2D.h
		a2=ik21*temp2D.x+ik22*temp2D.y+ik23*temp2D.h
		a3=ik31*temp2D.x+ik32*temp2D.y+ik33*temp2D.h
		a4=1.

		ir11=camera.rt11
		ir12=camera.rt21
		ir13=camera.rt31
		ir14=0.
		ir21=camera.rt12
		ir22=camera.rt22
		ir23=camera.rt32
		ir24=0.
		ir31=camera.rt13
		ir32=camera.rt23
		ir33=camera.rt33
		ir34=0.
		ir41=0.
		ir42=0.
		ir43=0.
		ir44=1.

		b1=ir11*a1+ir12*a2+ir13*a3+ir14*a4
		b2=ir21*a1+ir22*a2+ir23*a3+ir24*a4
		b3=ir31*a1+ir32*a2+ir33*a3+ir34*a4
		b4=ir41*a1+ir42*a2+ir43*a3+ir44*a4 

		it11=1.
		it12=0.
		it13=0.
		it14=camera.position.x
		it21=0.
		it22=1.
		it23=0.
		it24=camera.position.y
		it31=0.
		it32=0.
		it33=1.
		it34=camera.position.z
		it41=0.
		it42=0.
		it43=0.
		it44=1.

		punto3D.x=it11*b1+it12*b2+it13*b3+it14*b4
		punto3D.y=it21*b1+it22*b2+it23*b3+it24*b4
		punto3D.z=it31*b1+it32*b2+it33*b3+it34*b4
		punto3D.h=it41*b1+it42*b2+it43*b3+it44*b4

		if (punto3D.h!=0.):
			punto3D.x=punto3D.x/punto3D.h
			punto3D.y=punto3D.y/punto3D.h
			punto3D.z=punto3D.z/punto3D.h
			punto3D.h=1.
			output=1
		else:
			output=0

	return(output, punto3D)


def pixel2optical(p2d):
    aux = p2d.x
    p2d.x = LARGO_IMAGEN - 1 - p2d.y
    p2d.y = aux
    p2d.h = 1
    return p2d

# Crear una instancia de la cámara con los parámetros adecuados
mi_camara = Camera(k11, k12, k22, k23, k13, rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33, posicion_camara)

# Crear un punto en 2D
punto_2d = Punto2D(x, y, 1)

# Convertir el punto en 2D a coordenadas ópticas
punto_2d_optico = pixel2optical(punto_2d)

# Llamar a la función backproject
output, punto_3d = backproject(punto_2d_optico, mi_camara)

# Verificar si la proyección fue exitosa
if output == 1:
    # Acceder a las coordenadas en 3D desde el objeto Punto3D
    x_3d = punto_3d.x
    y_3d = punto_3d.y
    z_3d = punto_3d.z
else:
    print("La proyección no fue exitosa.")
