# Coordenadas conocidas en el primer sistema
x1, y1 = 310.99, 230.93

# Coordenadas correspondientes en el segundo sistema
x2, y2 = 0, 480

# Calcular la pendiente (m) y el desplazamiento (b)
m = (y2 - y1) / (x2 - x1)
b = y1 - m * x1

# Función para convertir coordenadas del primer sistema al segundo sistema
def convert_coordinates(x, y):
    new_y = m * x + b
    return new_y

# Ejemplo de conversión de coordenadas
x_primero = 310.99
y_primero = 230.93
x_segundo = 0
y_segundo = 480

# Convertir las coordenadas del primer sistema al segundo sistema
nuevo_y = convert_coordinates(x_primero, y_primero)

print("Coordenadas convertidas:", nuevo_y)
