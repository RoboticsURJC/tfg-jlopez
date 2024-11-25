from pibotj_rr.projGeom import *
import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import signal
import sys
import numpy 
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

ANCHO_IMAGEN = 640
LARGO_IMAGEN = 480
FX = 497.66
FY = 502.16
CX = 325.3
CY = 240.18

DEGTORAD = 3.1415926535897932 / 180
myCamera = None

class CameraPinHoleV2Class(Node):
    def __init__(self):
        super().__init__('camera_pinholev2_node')

        # tamaño de la cola de mensajes
        self.queueSize = 10
        self.subscription = self.create_subscription(
            Polygon, 
            'pothole_coords',
            self.coords_callback,
            self.queueSize)

        self.loadCamera()

        # Publicador del area calculado 
        self.area_publisher = self.create_publisher(Float32, 'area_calculated', self.queueSize)
        
        # Publicador del area calculado 
        self.min_coords_publisher = self.create_publisher(Pose2D, 'min_coords', self.queueSize)

        # Signal handler for cleanup
        signal.signal(signal.SIGINT, self.signal_handler)

    def coords_callback(self, coords):

        if not coords.points or len(coords.points) == 0:
            posenone = Pose2D()
            posenone.x = 0.0
            posenone.y = 0.0
            self.min_coords_publisher.publish(posenone)
            return

        # Calcula la conversión de los puntos de coordenadas
        # de la cámara en coordenadas del mundo real 
        array3D = []
        for i, point in enumerate(coords.points):
            pixel = Punto2D()
            pixel3D = Punto3D()

            # convertir las coordenadas del sistema de 192x192 
            # en imágenes de 640x480
            pixel.x = point.x*640/192
            pixel.y = point.y*480/192
            pixel.h = 1

            pixel3D = self.getIntersectionZ(pixel)
            #print(pixel3D.x, pixel3D.y)
            array3D.append(pixel3D)
        
        # importante añadir en las coordenadas la primera de todas al final
        array3D.append(array3D[0])

        # publica la coordenada que está detectada más cerca del robot
        pose = Pose2D()
        pose = self.get_min_coords(array3D)
        print(pose)
        self.min_coords_publisher.publish(pose)

        area = 0
        n = len(array3D)

        # Calcular el área usando el método shoelace 
        for i in range(n):

            x1 = array3D[i].x
            y1 = array3D[i].y

            x2 = array3D[(i + 1) % n].x
            y2 = array3D[(i + 1) % n].y

            area += (x1 * y2) - (y1 * x2)

        msg = Float32()
        msg.data = abs(area) / 2
        self.area_publisher.publish(msg)

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, shutting down...')
        sys.exit(0)  # Exit gracefully

    def loadCamera(self):
        global myCamera
        myCamera = PinholeCamera()
        #thetaY = 50*DEGTORAD # considerando que la camara (en vertical) está rotada 50º sobre eje Y
        thetaY = 40*DEGTORAD # considerando que la camara (en vertical) está rotada 50º sobre eje Y
        thetaZ = 0*DEGTORAD # considerando que la camara (en vertical) está rotada 50º sobre eje Y
        thetaX = 0*DEGTORAD # considerando que la camara (en vertical) está rotada 50º sobre eje Y

        R_y = numpy.array ([(numpy.cos(thetaY),0,-numpy.sin(thetaY)),(0,1,0),(numpy.sin(thetaY),0,numpy.cos(thetaY))]) # R is a 3x3 rotation matrix
        R_z = numpy.array ([(numpy.cos(thetaZ),-numpy.sin(thetaZ),0),(numpy.sin(thetaZ),numpy.cos(thetaZ),0),(0,0,1)]) # R is a 3x3 rotation matrix
        R_x = numpy.array ([(1,0,0),(0,numpy.cos(thetaX),numpy.sin(thetaX)),(0, -numpy.sin(thetaX),numpy.cos(thetaX))]) # R is a 3x3 rotation matrix

        R_subt = numpy.dot (R_y, R_z)
        R_tot = numpy.dot (R_subt, R_x)

        #T = numpy.array ([(1,0,0,0),(0,1,0,0),(0,0,1,-110)]) # T is a 3x4 traslation matrix
        T = numpy.array ([(1,0,0,0),(0,1,0,0),(0,0,1,-88)]) # T is a 3x4 traslation matrix

        Res = numpy.dot (R_tot,T)
        RT = numpy.append(Res, [[0,0,0,1]], axis=0) # RT is a 4x4 matrix
        K = numpy.array ([(FX,0,CX,0),(0,FY,CY,0),(0,0,1,0)]) # K is a 3x4 matrix
        # -------------------------------------------------------------

        # LOADING BOTH CAMERA MODELS JUST TO TEST THEM
        # -------------------------------------------------------------
        # A) PROGEO CAMERA
        # -------------------------------------------------------------
        myCamera.position.x = 0
        myCamera.position.y = 0
        myCamera.position.z = -88
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

        myCamera.fdistx = K[0,0] 
        myCamera.fdisty = K[1,1] 
        myCamera.u0 = K[0,2] 
        myCamera.v0 = K[1,2] 
        myCamera.rows = LARGO_IMAGEN
        myCamera.columns = ANCHO_IMAGEN

    def pixel2optical(self, p2d):
        aux = p2d.x
        p2d.x = LARGO_IMAGEN-1-p2d.y
        p2d.y = aux
        p2d.h = 1

        return p2d
        
    def getIntersectionZ(self, p2d):
        p3d = Punto3D ()
        res = Punto3D ()
        p2d_ = Punto2D ()

        x = myCamera.position.x
        y = myCamera.position.y
        z = myCamera.position.z

        p2d_ = self.pixel2optical(p2d)
        result, p3d = backproject(p2d_, myCamera)

        # Check division by zero
        if((p3d.z-z) == 0.0):
            res.h = 0.0
            return

        # Quiero que intersecte con el Plano Z = 0 
        zfinal = 0.0

        # Linear equation (X-x)/(p3d.X-x) = (Y-y)/(p3d.Y-y) = (Z-z)/(p3d.Z-z)
        xfinal = x + (p3d.x - x)*(zfinal - z)/(p3d.z-z)
        yfinal = y + (p3d.y - y)*(zfinal - z)/(p3d.z-z)	

        res.x = xfinal
        res.y = yfinal
        res.z = zfinal
        res.h = 1.0

        return res

    def get_min_coords(self, array):

        pose = Pose2D()

        min_x = 0
        min_y = 0
        for i in range(len(array)):

            if (min_x == 0):
                min_x = array[i].x
                min_y = array[i].y
            else:
                if (array[i].x < min_x):
                    min_x = array[i].x
                    min_y = array[i].y

        pose.x = min_x
        pose.y = min_y
        pose.theta = 0.0

        return pose


def main(args=None):
    rclpy.init(args=args)
    publisherObject = CameraPinHoleV2Class()
    try:
        rclpy.spin(publisherObject)
    except KeyboardInterrupt:
        publisherObject.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        publisherObject.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
