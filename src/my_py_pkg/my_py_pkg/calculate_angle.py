import numpy as np

def base_angel(punto_objetivo):


    vector_objetivo = punto_objetivo

    # Ángulo en el plano XY
    angulo_xy = np.arctan2(vector_objetivo[1], vector_objetivo[0])

    # Convertir el ángulo a grados
    angulo_xy_grados = np.degrees(angulo_xy)

    print(f"El ángulo para la base es : {angulo_xy_grados} grados")
    return angulo_xy_grados



def angulo_entre_lineas(normal, recta):
    # Calcular el producto escalar y las magnitudes
    dot_product = np.dot(normal, recta)
    magnitude1 = np.linalg.norm(normal)
    magnitude2 = np.linalg.norm(recta)

    # Calcular el ángulo en radianes
    alpha = np.arccos(dot_product / (magnitude1 * magnitude2))

    # Convertir el ángulo a grados si es necesario
    alpha_degrees = np.degrees(alpha)

    alpha_degrees -= 90

    if recta[2] >= 0:
        alpha_degrees += 180


    print(f"El ángulo para el hombro es: {alpha_degrees} grados")
    return alpha_degrees

def hombro_angle(punto):
    # Vector normal al plano (ejemplo: a lo largo de x)
    vector_normal = np.array([1, 0, 0])
    vector_recta = punto

    angle = angulo_entre_lineas(vector_normal,vector_recta)
    return angle


dato1 = ['RightShoulder', 0.040897217, -0.15468489, -0.5584, 1.0, 0.0, 0.0, 0.0]
dato2 = ['RightArm', 0.035634425, -0.41940695, -0.54728657, 0.6743311, 0.007569223, 0.00384615, -0.73839927]

# Extraer valores de posición de dato1
posicion_dato1 = dato1[1:4]  # Elementos 1 al 3 (0-indexed)
# Extraer valores de posición de dato2
posicion_dato2 = dato2[1:4]  # Elementos 1 al 3 (0-indexed)

# Restar los elementos correspondientes
punto_origen = [pos1 - pos2 for pos1, pos2 in zip(posicion_dato1, posicion_dato1)]
punto_objetivo = [pos2 - pos1 for pos1, pos2 in zip(posicion_dato1, posicion_dato2)]

# Mostrar resultados
print("Dato1:", punto_origen)
print("Dato2:", punto_objetivo)
base_angel(punto_objetivo)
hombro_angle(punto_objetivo)