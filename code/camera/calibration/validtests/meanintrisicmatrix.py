import numpy as np

# Lista para almacenar las matrices
matrices = []

# Leer y procesar los 10 archivos
for i in range(1, 11):
    filename = f"calibration_data{i}.txt"
    
    # Leer el archivo y procesar el contenido
    with open(filename, "r") as file:
        # Leer cada línea del archivo y dividirla en valores
        lines = file.readlines()
        # Convertir cada línea en una lista de flotantes
        matrix = [[float(val) for val in line.split()] for line in lines]
        # Convertir la lista de listas en una matriz numpy
        matrix_np = np.array(matrix)
        matrices.append(matrix_np)

# Calcular la media de las matrices
mean_matrix = np.mean(matrices, axis=0)

# Mostrar la media de las matrices
print("Media de las matrices:")
print(mean_matrix)
