import numpy as np

# Lista para almacenar las matrices
matrices = []

# Leer y procesar los 10 archivos
for i in range(1, 11):
    filename = f"calibration_data{i}.txt"
    
    # Leer el archivo y convertir el contenido a una matriz numpy
    with open(filename, "r") as file:
        content = file.read()
        # Convertir el contenido a matriz numpy
        matrix = np.array(eval(content))  # Usamos eval() para convertir la cadena a una lista y luego a una matriz
        matrices.append(matrix)

# Calcular la media de las matrices
mean_matrix = np.mean(matrices, axis=0)

# Mostrar la media de las matrices
print("Media de las matrices:")
print(mean_matrix)
