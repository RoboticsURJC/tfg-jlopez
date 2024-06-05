import numpy as np
import cv2
import glob

# Dimensiones del tablero de ajedrez
cb_width = 9
cb_height = 6
# Tamaño del cuadrado del tablero en milímetros 
cb_square_size = 26.7

# Se para la iteración cuando se encuentre el criterio 
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Puntos 3D del tablero de ajedrez 
cb_3D_points = np.zeros((cb_width *cb_height, 3), np.float32)
cb_3D_points[:,:2] = np.mgrid[0:cb_width, 0:cb_height].T.reshape(-1,2) * cb_square_size

# Lista para almacenar todos los puntos 3D (mundo real)
list_cb_3d_points = []
# Lista para almacenar todos los puntos 2D (imagen)
list_cb_2d_img_points = []
# Se consideran imágenes todas aquellas que acaban en '.jpg'
list_images = glob.glob('*.jpg')

# Iterar sobre todas las imágenes
for frame_name in list_images:

    # Lee la iamgen y se convierte en la escala de grises 
    img = cv2.imread(frame_name)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Dentro de la imagen de grises, encontrar las esquinas del tablero de ajedrez
    ret, corners = cv2.findChessboardCorners(gray,(9,6), None)

    # Si se encuentran, se añaden a los puntos 3D y 2D
    if ret == True:
        
        list_cb_3d_points.append(cb_3D_points)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
        list_cb_2d_img_points.append(corners2)

        # Dibuja y muestra las esquinas en la imagen  
        cv2.drawChessboardCorners(img, (cb_width, cb_height), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
        
cv2.destroyAllWindows()

# Después de procesar todas la imágenes,
# obtener la matriz de calibración y los coeficientes de distorsión
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(list_cb_3d_points, list_cb_2d_img_points, gray.shape[::-1], None, None)

print("Calibration Matrix: ")
print(mtx)
print("Distorsion: " , dist)

# Almacena el valor de la matriz de intrínsecos en un .txt
file = open("calibration_data.txt", "w+")

# Saving the array in a text file
content = str(mtx)
content2 = str(dist)
file.write(content)
file.write(content2)
file.close()
