import numpy as np

pi = np.pi

def base_angel(punto_objetivo):
    vector_referencia = np.array([0, 0, -1])  # Dirección de referencia (eje Z)

    # Normalizar los vectores para asegurarse de que tengan magnitudes de 1
    vector_normalizado = punto_objetivo / np.linalg.norm(punto_objetivo)
    vector_referencia_normalizado = vector_referencia / np.linalg.norm(vector_referencia)

    # Calcular el producto punto entre los dos vectores
    producto_punto = np.dot(vector_normalizado, vector_referencia_normalizado)

    # Calcular el ángulo entre los vectores usando la función inversa del coseno (arccos)
    angulo_apertura_radianes = np.arccos(producto_punto)

    angulo_apertura_radianes += 1.5708

    if (punto_objetivo[1] < 0):
        angulo_apertura_radianes -= pi
        if (punto_objetivo[2] < 0):
            angulo_apertura_radianes = -angulo_apertura_radianes
        if (punto_objetivo[2] > 0):
            angulo_apertura_radianes = -angulo_apertura_radianes

    # Convertir el ángulo a grados y ajustamos angulos.
    # if angulo_apertura_radianes > (3/2)*pi:
    #     angulo_apertura_radianes = (3/2)*pi
    # if angulo_apertura_radianes < -(1/2)*pi:
    #     angulo_apertura_radianes = -(1/2)*pi

    angulo_apertura_grados = np.degrees(angulo_apertura_radianes)

    print(f"Punto objetivo: {punto_objetivo}")
    print(f"El ángulo de apertura es: {angulo_apertura_grados} grados")

    return angulo_apertura_radianes



def angulo_entre_lineas(normal, recta):
    # Calcular el producto escalar y las magnitudes
    dot_product = np.dot(normal, recta)
    magnitude1 = np.linalg.norm(normal)
    magnitude2 = np.linalg.norm(recta)

    # Calcular el ángulo en radianes
    alpha = np.arccos(dot_product / (magnitude1 * magnitude2))

    alpha -= 1.5708
    alpha = -alpha

    if recta[0] > 0:
        alpha = 0
        alpha_degrees = 0

    # Convertir el ángulo a grados si es necesario
    alpha_degrees = np.degrees(alpha)
    if alpha_degrees > 90:
        alpha = (1/2)*pi 
        alpha_degrees = 90
    
    print(f"El ángulo para el hombro es: {alpha_degrees} grados")
    
    return alpha

def hombro_angle(punto):
    # Vector normal al plano (ejemplo: a lo largo de x)
    vector_normal = np.array([-1, 0, 0])
    vector_recta = punto

    print(f"Punto objetivo: {punto}")
    angle = angulo_entre_lineas(vector_normal,vector_recta)
    return angle


# angulo = base_angel((0, -0.25, 1))
# angulo = hombro_angle((-1,0,0))