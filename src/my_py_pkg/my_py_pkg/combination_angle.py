import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def obtener_direccion_recta(rotacion_x, rotacion_y, rotacion_z):
    # Convertir ángulos a radianes
    angulo_x = np.radians(rotacion_x)
    angulo_y = np.radians(rotacion_y)
    angulo_z = np.radians(rotacion_z)

    # Definir un vector de referencia, por ejemplo, el eje Z
    vector_referencia = np.array([0, 0, 1])

    # Calcular las matrices de rotación en X, Y y Z
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(angulo_x), -np.sin(angulo_x)],
                    [0, np.sin(angulo_x), np.cos(angulo_x)]])

    R_y = np.array([[np.cos(angulo_y), 0, np.sin(angulo_y)],
                    [0, 1, 0],
                    [-np.sin(angulo_y), 0, np.cos(angulo_y)]])

    R_z = np.array([[np.cos(angulo_z), -np.sin(angulo_z), 0],
                    [np.sin(angulo_z), np.cos(angulo_z), 0],
                    [0, 0, 1]])

    # Calcular la matriz de rotación total
    R_total = np.dot(np.dot(R_x, R_y), R_z)

    # Obtener la dirección de la recta transformando el vector de referencia
    direccion_recta = np.dot(R_total, vector_referencia)

    return direccion_recta

def visualizar_rectas(direcciones_rectas):
    # Crear una figura 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Origen de las rectas
    origen = np.zeros((3, len(direcciones_rectas)))

    # Graficar las rectas
    for i, direccion_recta in enumerate(direcciones_rectas):
        ax.quiver(*origen[:, i], *direccion_recta, color='r', label=f'Recta {i + 1}')

    # Configuraciones adicionales para la visualización
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Mostrar la leyenda
    ax.legend()

    # Mostrar el gráfico
    plt.show()

def calcular_angulo_entre_rectas(recta1, recta2):
    # Normalizar los vectores
    recta1_norm = recta1 / np.linalg.norm(recta1)
    recta2_norm = recta2 / np.linalg.norm(recta2)

    # Calcular el producto punto entre los vectores normalizados
    producto_punto = np.dot(recta1_norm, recta2_norm)

    # Calcular el ángulo en radianes
    angulo_radianes = np.arccos(np.clip(producto_punto, -1.0, 1.0))

    # Convertir el ángulo a grados
    angulo_grados = np.degrees(angulo_radianes)

    print ("Angulo de ambas rectas" + str(angulo_grados))

# Ejemplo de uso con dos direcciones de rectas
rotacion_x_1, rotacion_y_1, rotacion_z_1 = -72, 63, 22
rotacion_x_2, rotacion_y_2, rotacion_z_2 = 29, 76, -17

direccion_recta_1 = obtener_direccion_recta(rotacion_x_1, rotacion_y_1, rotacion_z_1)
direccion_recta_2 = obtener_direccion_recta(rotacion_x_2, rotacion_y_2, rotacion_z_2)

calcular_angulo_entre_rectas(direccion_recta_1, direccion_recta_2)

visualizar_rectas([direccion_recta_1, direccion_recta_2])
