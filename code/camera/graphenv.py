import matplotlib.pyplot as plt

# Coordenadas originales del píxel
x_pixel = 0
y_pixel = 0

# Tamaño de la imagen en píxeles
ancho_imagen = 640
altura_imagen = 480

# Calcular coordenadas relativas al centro de la imagen
x_imagen_centro = x_pixel - ancho_imagen / 2
y_imagen_centro = altura_imagen / 2 - y_pixel

# Convertir coordenadas relativas en coordenadas de imagen absolutas
x_imagen = -x_imagen_centro + ancho_imagen / 2
y_imagen = y_imagen_centro + altura_imagen / 2

# Crear la figura y los ejes
fig, ax = plt.subplots()

# Dibujar la imagen
#ax.imshow([[0, 0], [0, 0]], extent=[0, ancho_imagen, 0, altura_imagen], alpha=0)

# Dibujar las coordenadas de píxel
ax.plot(x_imagen_centro, y_imagen_centro, 'go', label='Coordenadas centro')

ax.plot(x_pixel, y_pixel, 'ro', label='Coordenadas de Píxel')


# Dibujar las coordenadas de imagen
ax.plot(x_imagen, y_imagen, 'bo', label='Coordenadas de Imagen')

ax.plot(y_imagen, x_imagen, 'yo', label='Coordenadas de Imagen invertidas')

# Mostrar la leyenda
ax.legend()

# Ajustar los límites de los ejes
#ax.set_xlim(0, ancho_imagen)
#ax.set_ylim(0, altura_imagen)

# Etiquetas de los ejes
ax.set_xlabel('Coordenada X')
ax.set_ylabel('Coordenada Y')

# Título del gráfico
ax.set_title('Conversión de Coordenadas')

# Mostrar el gráfico
plt.grid(True)
plt.gca().invert_yaxis()
plt.show()

