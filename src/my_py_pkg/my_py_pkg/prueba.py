# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# def plot_points_with_lines(points):
#     fig = plt.figure(figsize=(8, 8))
#     ax = fig.add_subplot(111, projection='3d')

#     # Dibujar el sistema de referencia
#     ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
#     ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
#     ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')

#     # Extraer las coordenadas x, y, z de los puntos
#     points = np.array(points)
#     x_coords, y_coords, z_coords = points[:, 0], points[:, 1], points[:, 2]

#     # Dibujar los puntos
#     ax.scatter(x_coords, y_coords, z_coords, c='black', marker='o', label='Puntos')

#     # Dibujar líneas que conectan los puntos
#     for i in range(len(points) - 1):
#         ax.plot([points[i, 0], points[i + 1, 0]], [points[i, 1], points[i + 1, 1]], [points[i, 2], points[i + 1, 2]], color='gray', linestyle='--')

#     # Configurar límites
#     max_range = np.array([x_coords.max()-x_coords.min(), y_coords.max()-y_coords.min(), z_coords.max()-z_coords.min()]).max() / 2.0

#     mid_x = (x_coords.max()+x_coords.min()) * 0.5
#     mid_y = (y_coords.max()+y_coords.min()) * 0.5
#     mid_z = (z_coords.max()+z_coords.min()) * 0.5

#     ax.set_xlim(mid_x - max_range, mid_x + max_range)
#     ax.set_ylim(mid_y - max_range, mid_y + max_range)
#     ax.set_zlim(mid_z - max_range, mid_z + max_range)

#     # Etiquetas de los ejes
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')

#     # Mostrar leyenda
#     ax.legend()

#     # Mostrar el gráfico
#     plt.show()

# # Ejemplo de uso
# dato1 = ['RightShoulder', 0.040897217, -0.15468489, -0.5584, 1.0, 0.0, 0.0, 0.0]
# dato2 = ['RightArm', 0.035634425, -0.41940695, -0.54728657, 0.6743311, 0.007569223, 0.00384615, -0.73839927]
# dato3 = ['RightForeArm', 0.10053623, -0.6650806, -0.60245514, 0.69210124, -0.015738446, 0.12846142, -0.71012235]
# dato5 = ['RightHand', 0.09384858, -0.83751094, -0.63166445, 0.69633347, 0.03521535, 0.02849228, -0.716307]

# # Crear un nuevo array con las posiciones 1 a 3 de cada dato
# puntos = [[dato[1], dato[2], dato[3]] for dato in [dato1, dato2, dato3, dato5]]

# primer_dato = puntos[0]
# puntos = [[dato[0] - primer_dato[0], dato[1] - primer_dato[1], dato[2] - primer_dato[2]] for dato in puntos]

# print(puntos)

# # plot_points_with_lines(puntos)
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R


def degrees_from_axis_x(punto1, punto2):

    # Dirección de la recta
    direccion_recta = punto2 - punto1
    direccion_recta /= np.linalg.norm(direccion_recta)  # Normalizar el vector

    # Vector normal al plano a lo largo del eje x
    vector_normal = np.array([1, 0, 0])

    # Calcular el coseno del ángulo entre la dirección de la recta y el vector normal al plano
    cos_angulo = np.dot(direccion_recta, vector_normal)

    # Calcular el ángulo en radianes
    angulo_radianes = np.arccos(cos_angulo)

    # Convertir el ángulo a grados
    angulo_grados = np.degrees(angulo_radianes)

    print("El angulo formado entre el hombro y el brazo: " + str(angulo_grados))

def plot_points_and_quaternions_separate_directions(points, quaternions):
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Dibujar el sistema de referencia
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')

    # Extraer las coordenadas x, y, z de los puntos
    points = np.array(points)
    x_coords, y_coords, z_coords = points[:, 0], points[:, 1], points[:, 2]

    # Dibujar los puntos
    ax.scatter(x_coords, y_coords, z_coords, c='black', marker='o', label='Puntos')

    # Dibujar líneas entre los puntos
    for i in range(len(points)-1):
        ax.plot([points[i, 0], points[i+1, 0]], [points[i, 1], points[i+1, 1]], [points[i, 2], points[i+1, 2]], color='gray', linestyle='--')

    # Configurar límites
    max_range = np.array([x_coords.max()-x_coords.min(), y_coords.max()-y_coords.min(), z_coords.max()-z_coords.min()]).max() / 2.0

    mid_x = (x_coords.max()+x_coords.min()) * 0.5
    mid_y = (y_coords.max()+y_coords.min()) * 0.5
    mid_z = (z_coords.max()+z_coords.min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Etiquetas de los ejes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Mostrar leyenda
    ax.legend()

    # Dibujar flechas para representar cuaterniones
    for i, quaternion in enumerate(quaternions):
        r = R.from_quat(quaternion)

        # Representar cada dirección por separado
        for axis in ['x', 'y', 'z']:
            arrow_length = 0.2
            arrow_start = points[i]
            arrow_end = arrow_start + arrow_length * r.apply([1 if a == axis else 0 for a in ['x', 'y', 'z']])

            ax.quiver(arrow_start[0], arrow_start[1], arrow_start[2],
                      arrow_end[0], arrow_end[1], arrow_end[2],
                      color='orange', arrow_length_ratio=0.1)


    degrees_from_axis_x(points[0], points[1])

    # Mostrar el gráfico
    plt.show()


# Ejemplo de uso
dato1 = ['RightShoulder', 0.040897217, -0.15468489, -0.5584, 1.0, 0.0, 0.0, 0.0]
dato2 = ['RightArm', 0.035634425, -0.41940695, -0.54728657, 0.6743311, 0.007569223, 0.00384615, -0.73839927]
dato3 = ['RightForeArm', 0.10053623, -0.6650806, -0.60245514, 0.69210124, -0.015738446, 0.12846142, -0.71012235]
dato4 = ['RightHand', 0.09384858, -0.83751094, -0.63166445, 0.69633347, 0.03521535, 0.02849228, -0.716307]

# Crear un nuevo array con las posiciones 1 a 3 de cada dato
puntos = [[dato[1], dato[2], dato[3]] for dato in [dato1, dato2, dato3, dato4]]

cuaterniones = [[dato[4], dato[5], dato[6], dato[7]] for dato in [dato1, dato2, dato3, dato4]]

primer_dato = puntos[0]
puntos = [[dato[0] - primer_dato[0], dato[1] - primer_dato[1], dato[2] - primer_dato[2]] for dato in puntos]

print(puntos)

plot_points_and_quaternions_separate_directions(puntos, cuaterniones)
