###############################################################################
# (C) 2019 - Julio Vega
###############################################################################
# Algoritmo de navegación de PiBot basado en sonar visual percibido por PiCam.#
# En este programa, los motores están parados, para así poder hacer pruebas.  #
###############################################################################

# -*- coding: utf-8 -*-
#from progeo import *
from projGeom import *
import cv2
import pygame
import timeit
import time
import numpy

# defino variables globales:
ANCHO_IMAGEN = 320
LARGO_IMAGEN = 240

# PYGAME CONSTANTS
pyGameFont = None
pyGameScreen = None # pantalla donde dibujo
ANCHO_ESCENA = 700
LARGO_ESCENA = 700
BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)
GREY =  (128, 128, 128)
LIGHT_BLUE = (135,206,250)
SCALE = 1 # los valores reales los dividimos entre este factor para dibujarlos

DEGTORAD = 3.1415926535897932 / 180
myCamera = None
originalImg = None
hsvImg = None
bnImg = None
groundImg = None
fronteraImg = None
GREEN_MIN = numpy.array([20, 50, 100],numpy.uint8)#numpy.array([48, 138, 138],numpy.uint8)
GREEN_MAX = numpy.array([90, 235, 210],numpy.uint8)#numpy.array([67, 177, 192],numpy.uint8)
puntosFrontera = 0

def dameMinDistancia ():
	p = Punto2D ()
	minDist = 99999
	for i in range(puntosFrontera):
		p.x = int(round(fronteraArray[i][0]))
		if (p.x > 0): # si no, no me interesa porque es un punto a mis espaldas, retroproyectado al horizonte
			p.y = int(round(fronteraArray[i][1]))

			dist = numpy.sqrt(p.x*p.x + p.y*p.y);

			if (dist != 0) and (dist < minDist):
				minDist = dist

	return int(round(minDist))

def pixel2optical (p2d):
	aux = p2d.x
	p2d.x = LARGO_IMAGEN-1-p2d.y
	p2d.y = aux
	p2d.h = 1

	return p2d

def optical2pygame (p2d):
	aux = LARGO_ESCENA-(p2d.x/SCALE)
	p2d.x = int(round(ANCHO_ESCENA/2))+(p2d.y/SCALE)
	p2d.y = aux
	p2d.h = 1

	return p2d

def initPyGame ():
	global pyGameScreen
	global pyGameFont

	# Initialize the game engine
	pygame.init()
	 
	# Set the height and width of the screen
	size = [ANCHO_ESCENA, LARGO_ESCENA]
	pyGameScreen = pygame.display.set_mode(size)

	# initialize font
	pyGameFont = pygame.font.SysFont("monospace", 15)
	 
	pygame.display.set_caption("LASER VISUAL")

	getCameraExtrems ()

def runPyGame (): 
	#Loop until the user clicks the close button.
	done = False
	clock = pygame.time.Clock()

	while not done:
		# This limits the while loop to a max of 10 times per second.
		# Leave this out and we will use all CPU we can.
		clock.tick(10)

		for event in pygame.event.get(): # User did something
			if event.type == pygame.QUIT: # If user clicked close
				done=True # Flag that we are done so we exit this loop

		# All drawing code happens after the for loop and but
		# inside the main while done==False loop.

		# Clear the screen and set the screen background
		pyGameScreen.fill(LIGHT_BLUE)

		drawCameraLimits ()
		drawFloor ()
		drawFrontera ()

		# Go ahead and update the screen with what we've drawn.
		# This MUST happen after all the other drawing commands.
		pygame.display.flip()
 
	# Be IDLE friendly
	pygame.quit()

def getIntersectionZ (p2d):
	p3d = Punto3D ()
	res = Punto3D ()
	p2d_ = Punto2D ()

	x = myCamera.position.x
	y = myCamera.position.y
	z = myCamera.position.z

	p2d_ = pixel2optical(p2d)
	result, p3d = backproject(p2d_, myCamera)

	# Check division by zero
	if((p3d.z-z) == 0.0):
		res.h = 0.0
		return

	zfinal = 0. # Quiero que intersecte con el Plano Z = 0

	# Linear equation (X-x)/(p3d.X-x) = (Y-y)/(p3d.Y-y) = (Z-z)/(p3d.Z-z)
	xfinal = x + (p3d.x - x)*(zfinal - z)/(p3d.z-z)
	yfinal = y + (p3d.y - y)*(zfinal - z)/(p3d.z-z)	

	res.x = xfinal
	res.y = yfinal
	res.z = zfinal
	res.h = 1.0

	return res

def drawCameraLimits ():
	p1 = Punto2D()
	p2 = Punto2D()
	p3 = Punto2D()
	p4 = Punto2D()

	p1.x = cameraLimits[0][0]
	p1.y = cameraLimits[0][1]
	p2.x = cameraLimits[1][0]
	p2.y = cameraLimits[1][1]
	p3.x = cameraLimits[2][0]
	p3.y = cameraLimits[2][1]
	p4.x = cameraLimits[3][0]
	p4.y = cameraLimits[3][1]

	p1 = optical2pygame(p1)
	p2 = optical2pygame(p2)
	p3 = optical2pygame(p3)
	p4 = optical2pygame(p4)

	pygame.draw.lines(pyGameScreen, WHITE, False, [[p1.x, p1.y], [p3.x, p3.y], [p4.x, p4.y], [p2.x, p2.y], [p1.x, p1.y]], 2)

def drawFloor ():
	i = 0
	greenTurn = True

	while (i < LARGO_ESCENA*SCALE):
		if (greenTurn):
			label = pyGameFont.render(str((LARGO_ESCENA-(i/SCALE))*SCALE)+"mm", 1, BLUE)
			pygame.draw.line(pyGameScreen, BLUE, [0, i/SCALE], [ANCHO_ESCENA,i/SCALE], 1)
			pyGameScreen.blit(label, (0, i/SCALE))
			greenTurn = False

		else:
			label = pyGameFont.render(str((LARGO_ESCENA-(i/SCALE))*SCALE)+"mm", 1, GREY)
			pygame.draw.line(pyGameScreen, GREY, [0, i/SCALE], [ANCHO_ESCENA,i/SCALE], 1)
			pyGameScreen.blit(label, (ANCHO_ESCENA-45, i/SCALE))
			greenTurn = True

		i = i + 50

def drawFrontera ():
	p2d = Punto2D()
	p2d_ = Punto2D()
	for i in range(puntosFrontera):
		#redondeamos esos valores decimales obtenidos de la retroproyección
		p2d.x = int(round(fronteraArray[i][0]))
		p2d.y = int(round(fronteraArray[i][1]))
		p2d_ = optical2pygame (p2d)
		#print "Frontera [",fronteraArray[i][0],",",fronteraArray[i][1],"] en PyGame[",p2d_.x,",",p2d_.y,"]"
		#print "PyGame[",p2d_.x,",",p2d_.y,"]"
		pygame.draw.circle(pyGameScreen, RED, [p2d_.x, p2d_.y], 1)

def getFronteraImage ():
	global fronteraImg
	global puntosFrontera

	pixel = Punto2D()
	pixelOnGround3D = Punto3D()
	tmp2d = Punto2D()
	j = 0
	puntosFrontera = 0

	# Ground image: recorremos la imagen de abajo a arriba (i=filas) y de izquierda a derecha (j=columnas)
	while (j < ANCHO_IMAGEN): # recorrido en columnas
		i = LARGO_IMAGEN-1
		esFrontera = None
		while ((i>=0) and (esFrontera == None)): # recorrido en filas
			pos = i*ANCHO_IMAGEN+j # posicion actual

			pix = bnImg[i, j] # value 0 or 255 (frontera)

			if (pix != 0): # si no soy negro
				esFrontera = True # lo damos por frontera en un principio, luego veremos
				# Calculo los demás vecinos, para ver de qué color son...
				c = j - 1
				row = i
				v1 = row*ANCHO_IMAGEN+c

				if (not((c >= 0) and (c < ANCHO_IMAGEN) and 
				(row >= 0) and (row < LARGO_IMAGEN))): # si no es válido ponemos sus valores a 0
					pix = 0
				else:
					pix = bnImg[row, c]
				'''
				if (pix == 0): # si no soy color campo, sigo comprobando
					c = j - 1
					row = i - 1
					v2 = row*ANCHO_IMAGEN+c

					if (not((c >= 0) and (c < ANCHO_IMAGEN) and 
					(row >= 0) and (row < LARGO_IMAGEN))): # si no es válido ponemos sus valores a 0
						pix = 0
					else:
						pix = bnImg[row, c]

					if (pix == 0): # si no soy color campo, sigo comprobando
						c = j
						row = i - 1
						v3 = row*ANCHO_IMAGEN+c

						if (not((c >= 0) and (c < ANCHO_IMAGEN) and 
						(row >= 0) and (row < LARGO_IMAGEN))): # si no es válido ponemos sus valores a 0
							pix = 0
						else:
							pix = bnImg[row, c]

						if (pix == 0): # si no soy color campo, sigo comprobando
							c = j + 1
							row = i - 1
							v4 = row*ANCHO_IMAGEN+c

							if (not((c >= 0) and (c < ANCHO_IMAGEN) and 
							(row >= 0) and (row < LARGO_IMAGEN))): # si no es válido ponemos sus valores a 0
								pix = 0
							else:
								pix = bnImg[row, c]

							if (pix == 0): # si no soy color campo, sigo comprobando
								c = j + 1
								row = i
								v5 = row*ANCHO_IMAGEN+c

								if (not((c >= 0) and (c < ANCHO_IMAGEN) and 
								(row >= 0) and (row < LARGO_IMAGEN))): # si no es válido ponemos sus valores a 0
									pix = 0
								else:
									pix = bnImg[row, c]

								if (pix == 0): # si no soy color campo, sigo comprobando
									c = j + 1
									row = i + 1
									v6 = row*ANCHO_IMAGEN+c

									if (not((c >= 0) and (c < ANCHO_IMAGEN) and 
									(row >= 0) and (row < LARGO_IMAGEN))): # si no es válido ponemos sus valores a 0
										pix = 0
									else:
										pix = bnImg[row, c]

									if (pix == 0): # si no soy color campo, sigo comprobando
										c = j
										row = i + 1
										v7 = row*ANCHO_IMAGEN+c

										if (not((c >= 0) and (c < ANCHO_IMAGEN) and 
										(row >= 0) and (row < LARGO_IMAGEN))): # si no es válido ponemos sus valores a 0
											pix = 0
										else:
											pix = bnImg[row, c]

										if (pix == 0): # si no soy color campo, sigo comprobando
											c = j - 1
											row = i + 1
											v8 = row*ANCHO_IMAGEN+c

											if (not((c >= 0) and (c < ANCHO_IMAGEN) and 
											(row >= 0) and (row < LARGO_IMAGEN))): # si no es válido ponemos sus valores a 0
												pix = 0
											else:
												pix = bnImg[row, c]

											if (pix == 0): # si no soy color campo, se acabó, no es pto. frontera
												esFrontera = None
				'''

				if (esFrontera == True): # si NO SOY COLOR CAMPO y alguno de los vecinitos ES color campo...
					pixel.x = j
					pixel.y = i
					pixel.h = 1
					fronteraImg[i,j] = 255

					# obtenemos su backproject e intersección con plano Z en 3D
					pixelOnGround3D = getIntersectionZ (pixel)

					# vamos guardando estos puntos frontera 3D para luego dibujarlos con PyGame
					fronteraArray[puntosFrontera][0] = pixelOnGround3D.x
					fronteraArray[puntosFrontera][1] = pixelOnGround3D.y
					puntosFrontera = puntosFrontera + 1
					#print "Hay frontera en pixel [",i,",",j,"] que intersecta al suelo en [",pixelOnGround3D.x,",",pixelOnGround3D.y,",",pixelOnGround3D.z,"]"

			i = i - 1
		j = j + 5

def loadCamera ():
	global myCamera
	myCamera = PinholeCamera()
	# -------------------------------------------------------------
	# LOADING MATRICES:
	# -------------------------------------------------------------
	#R = numpy.array ([(1,0,0),(0,1,0),(0,0,1)]) # R is a 3x3 rotation matrix
	#R = numpy.array ([(1,0,-0.0274),(0,1,0),(0.0274,0,1)]) # R is a 3x3 rotation matrix
	thetaY = 59*DEGTORAD # considerando que la camara (en vertical) está rotada 90º sobre eje Y
	thetaZ = 0*DEGTORAD # considerando que la camara (en vertical) está rotada 90º sobre eje Y
	thetaX = 0*DEGTORAD # considerando que la camara (en vertical) está rotada 90º sobre eje Y

	R_y = numpy.array ([(numpy.cos(thetaY),0,-numpy.sin(thetaY)),(0,1,0),(numpy.sin(thetaY),0,numpy.cos(thetaY))]) # R is a 3x3 rotation matrix
	R_z = numpy.array ([(numpy.cos(thetaZ),-numpy.sin(thetaZ),0),(numpy.sin(thetaZ),numpy.cos(thetaZ),0),(0,0,1)]) # R is a 3x3 rotation matrix
	R_x = numpy.array ([(1,0,0),(0,numpy.cos(thetaX),numpy.sin(thetaX)),(0, -numpy.sin(thetaX),numpy.cos(thetaX))]) # R is a 3x3 rotation matrix

	R_subt = numpy.dot (R_y, R_z)
	R_tot = numpy.dot (R_subt, R_x)

	T = numpy.array ([(1,0,0,0),(0,1,0,0),(0,0,1,-110)]) # T is a 3x4 traslation matrix
	Res = numpy.dot (R_tot,T)
	RT = numpy.append(Res, [[0,0,0,1]], axis=0) # RT is a 4x4 matrix
	K = numpy.array ([(313.89382026,0,117.5728043,0),(0,316.64906146,158.04145907,0),(0,0,1,0)]) # K is a 3x4 matrix
	# -------------------------------------------------------------

	# -------------------------------------------------------------
	# LOADING BOTH CAMERA MODELS JUST TO TEST THEM
	# -------------------------------------------------------------
	# A) PROGEO CAMERA
	# -------------------------------------------------------------
	myCamera.position.x = 0
	myCamera.position.y = 0
	myCamera.position.z = -110
	myCamera.position.h = 1

	# K intrinsec parameters matrix (values got from the PiCamCalibration.py)
	myCamera.k11 = K[0,0]
	myCamera.k12 = K[0,1]
	myCamera.k13 = K[0,2]
	myCamera.k14 = K[0,3]

	myCamera.k21 = K[1,0]
	myCamera.k22 = K[1,1]
	myCamera.k23 = K[1,2]
	myCamera.k24 = K[1,3]

	myCamera.k31 = K[2,0]
	myCamera.k32 = K[2,1]
	myCamera.k33 = K[2,2]
	myCamera.k34 = K[2,3]

	# RT rotation-traslation matrix
	myCamera.rt11 = RT[0,0]
	myCamera.rt12 = RT[0,1]
	myCamera.rt13 = RT[0,2]
	myCamera.rt14 = RT[0,3]

	myCamera.rt21 = RT[1,0]
	myCamera.rt22 = RT[1,1]
	myCamera.rt23 = RT[1,2]
	myCamera.rt24 = RT[1,3]

	myCamera.rt31 = RT[2,0]
	myCamera.rt32 = RT[2,1]
	myCamera.rt33 = RT[2,2]
	myCamera.rt34 = RT[2,3]

	myCamera.rt41 = RT[3,0]
	myCamera.rt42 = RT[3,1]
	myCamera.rt43 = RT[3,2]
	myCamera.rt44 = RT[3,3]

	myCamera.fdistx = K[0,0] #myCamera.k11
	myCamera.fdisty = K[1,1] #myCamera.k22
	myCamera.u0 = K[0,2] #myCamera.k13
	myCamera.v0 = K[1,2] #myCamera.k23
	myCamera.rows = LARGO_IMAGEN
	myCamera.columns = ANCHO_IMAGEN

	#myCamera.printCameraInfo ()
	# -------------------------------------------------------------
	'''
	# -------------------------------------------------------------
	# B) OPENCV CAMERA
	# -------------------------------------------------------------
	cameraCV = PinholeCamera_CV ()
	campositionCV = [(myCamera.position.x, myCamera.position.y, myCamera.position.z)]
	D = [(0.04853663), (-0.19172296), (-0.00088241), (-0.00245449), (-0.2196874)]
	K_CV = numpy.array ([(313.89382026,0,158.04145907),(0,316.64906146,117.5728043),(0,0,1)]) # K_CV is a 3x3 matrix
	cameraCV.setPinHoleCamera(K_CV, D, R, T, ANCHO_IMAGEN, LARGO_IMAGEN, campositionCV)

	cameraCV.printCameraInfo ()
	'''

def getCameraExtrems ():
	global cameraLimits
	p3d = Punto3D ()
	cameraPos3D = Punto3D ()
	p2d = Punto2D ()
	cameraLimits = numpy.zeros((4,2), dtype = "float64")
	# -------------------------------------------------------------
	# PRUEBAS DE PROJECT Y BACKPROJECT
	# -------------PROJECT-----------------------------------------
	p3d.x = 1000
	p3d.y = 0
	p3d.z = 0
	p3d.h = 1

	result, p2d = project(p3d,myCamera)
	#print "Segun progeo-project: el punto 3D[",p3d.x,",",p3d.y,",",p3d.z,"] proyecta en [",p2d.x,",",p2d.y,",",p2d.h,"]\n"

	# ------------BACKPROJECT--------------------------------------
	# Coordenadas en 3D de la posicion de la cámara
	cameraPos3D.x = myCamera.position.x
	cameraPos3D.y = myCamera.position.y
	cameraPos3D.z = myCamera.position.z
	cameraPos3D.h = 1

	# Retroproyectamos los puntos A, B, C, D de las esquinas de la
	# cámara, según punto de vista de la cámara. Como vemos, ignoramos
	# los puntos situados por encima del horizonte, para no liarnos
	# A-----------B-----------C
	# |                       |
	# |                       |
	# |                       |
	# |                       |
	# |                       |
	# |                       |
	# |                       |
	# D-----------E-----------F

	# A
	# -------------------------------------------------------------
	p2d.x = 0
	p2d.y = 0
	p2d.h = 1

	p3d = getIntersectionZ (p2d)
	cameraLimits[0][0] = p3d.x
	cameraLimits[0][1] = p3d.y
	#print "El punto A intersecta al plano suelo en [",p3d.x,",",p3d.y,",",p3d.z,"]"

	# B
	# -------------------------------------------------------------
	p2d.x = (ANCHO_IMAGEN-1)/2
	p2d.y = 0
	p2d.h = 1

	p3d = getIntersectionZ (p2d)

	#print "El punto B intersecta al plano suelo en [",p3d.x,",",p3d.y,",",p3d.z,"]"

	# C
	# -------------------------------------------------------------
	p2d.x = ANCHO_IMAGEN-1
	p2d.y = 0
	p2d.h = 1

	p3d = getIntersectionZ (p2d)
	cameraLimits[1][0] = p3d.x
	cameraLimits[1][1] = p3d.y
	#print "El punto C intersecta al plano suelo en [",p3d.x,",",p3d.y,",",p3d.z,"]"

	# D
	# -------------------------------------------------------------
	p2d.x = 0
	p2d.y = LARGO_IMAGEN-1
	p2d.h = 1

	p3d = getIntersectionZ (p2d)
	cameraLimits[2][0] = p3d.x
	cameraLimits[2][1] = p3d.y
	#print "El punto D intersecta al plano suelo en [",p3d.x,",",p3d.y,",",p3d.z,"]"

	# E
	# -------------------------------------------------------------
	p2d.x = (ANCHO_IMAGEN-1)/2
	p2d.y = LARGO_IMAGEN-1
	p2d.h = 1

	p3d = getIntersectionZ (p2d)

	#print "El punto E intersecta al plano suelo en [",p3d.x,",",p3d.y,",",p3d.z,"]"

	# F
	# -------------------------------------------------------------
	p2d.x = ANCHO_IMAGEN-1
	p2d.y = LARGO_IMAGEN-1
	p2d.h = 1

	p3d = getIntersectionZ (p2d)
	cameraLimits[3][0] = p3d.x
	cameraLimits[3][1] = p3d.y
	#print "El punto F intersecta al plano suelo en [",p3d.x,",",p3d.y,",",p3d.z,"]"

def testImageReading ():
	for j in range(ANCHO_IMAGEN): # recorrido en columnas
		i = LARGO_IMAGEN-1
		while (i>=0): # recorrido en filas
			pix = fronteraImg[i, j] # actual red pixel

			print ("Pix value =", pix)
			i = i - 1

def loadImages ():
	global originalImg
	global hsvImg
	global bnImg
	global fronteraImg
	global fronteraArray

	originalImg = cv2.imread ('imgs_static/incline/55H.png')
	hsvImg = cv2.cvtColor(originalImg,cv2.COLOR_BGR2HSV)
	bnImg = cv2.inRange(hsvImg, GREEN_MIN, GREEN_MAX)
	fronteraImg = numpy.zeros((LARGO_IMAGEN,ANCHO_IMAGEN), dtype = "uint8")
	fronteraArray = numpy.zeros((ANCHO_IMAGEN,2), dtype = "float64")

def showImages ():
	cv2.imshow('Original', originalImg)
	cv2.imshow('HSV', hsvImg)
	cv2.imshow('B/N', bnImg)
	cv2.imshow('Frontera', fronteraImg)

	# wait for ESC key
	k = cv2.waitKey()

	if k == 27: # If escape was pressed exit
		cv2.destroyAllWindows()

if __name__=="__main__":
	time1  = time.time()
	loadCamera()
	time2  = time.time()
	print ("Load Camera geometric model", time2 - time1)
	loadImages()
	time3  = time.time()
	print ("Load and filter image", time3 - time2)
	getFronteraImage()
	time4  = time.time()
	print ("Get Frontier", time4 - time3)

	#showImages ()
	#testImageReading ()
	initPyGame()
	runPyGame()
	print(dameMinDistancia())

