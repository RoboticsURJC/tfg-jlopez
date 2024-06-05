import numpy as np
import os

# Directorio actual
directorio_actual = os.getcwd()

# Lista para almacenar las matrices
matrices = []

# Leer matrices de archivos .txt en el directorio actual
for archivo in os.listdir(directorio_actual):
    if archivo.endswith(".txt"):
        with open(os.path.join(directorio_actual, archivo), 'r') as file:
            # Leer líneas del archivo
            lineas = file.readlines()
            matrices.append(lineas)

matrices_np = [np.array(matriz) for matriz in matrices]

suma_0_0 = 0
suma_0_1 = 0
suma_0_2 = 0
suma_1_0 = 0
suma_1_1 = 0
suma_1_2 = 0
suma_2_0 = 0
suma_2_1 = 0
suma_2_2 = 0
num_matrices = 0

for indice, matriz in enumerate(matrices_np):

    # cálculos primera fila 
    tokens_0 = matriz[0].split(',')
    suma_0_0 += float(tokens_0[0][2:])
    suma_0_1 += float(tokens_0[1])
    suma_0_2 += float(tokens_0[2][:-2])

    # cálculos segunda fila 
    tokens_1 = matriz[1].split(',')
    suma_1_0 += float(tokens_1[0][2:])
    suma_1_1 += float(tokens_1[1])
    suma_1_2 += float(tokens_1[2][:-2])

    # cálculos tercera fila
    tokens_2 = matriz[2].split(',')
    suma_2_0 += float(tokens_2[0][2:])
    suma_2_1 += float(tokens_2[1])
    suma_2_2 += float(tokens_2[2][:-3])

    num_matrices += 1

matriz_media = [[suma_0_0/num_matrices, suma_0_1/num_matrices, suma_0_2/num_matrices],
                [suma_1_0/num_matrices, suma_1_1/num_matrices, suma_1_2/num_matrices],
                [suma_2_0/num_matrices, suma_2_1/num_matrices, suma_2_2/num_matrices]]

print(matriz_media)
