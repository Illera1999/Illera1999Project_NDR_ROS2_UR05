import numpy as np

def angulo_recta_plano(punto_recta, cuaternion_director):
  """
  Calcula el ángulo que forma una recta con respecto a un plano.

  Args:
    punto_recta: Punto de la recta.
    cuaternion_director: Cuaternio que representa el vector director de la recta.

  Returns:
    Ángulo en grados.
  """

  # Normalizamos el vector normal al plano.
  vector_normal = np.array([0, 0, 1])
  vector_normal = vector_normal / np.linalg.norm(vector_normal)

  # Obtenemos la parte imaginaria del cuaternion que representa la recta.
  parte_imaginaria_recta = cuaternion_director.imag

  # Agregamos una dimensión adicional al punto_recta.
  punto_recta_expandido = np.expand_dims(punto_recta, axis=0)

  # Eliminamos la dimensión adicional del punto_recta_expandido.
  punto_recta_comprimido = np.squeeze(punto_recta_expandido)

  # Transmitimos el punto_recta a una forma de (4,).
  punto_recta_transmitido = np.broadcast_to(punto_recta_comprimido, (4,))

  # Obtenemos el vector director como la diferencia entre la parte imaginaria del cuaternion que representa el punto y la parte imaginaria del cuaternion que representa la recta.
  vector_director = parte_imaginaria_recta - punto_recta_transmitido

  # Calculamos el producto escalar entre el vector normal al plano y el vector director de la recta.
  producto_escalar = np.dot(vector_normal, vector_director)

  # Calculamos el ángulo entre dos vectores a partir de su producto escalar.
  angulo = np.arccos(producto_escalar)

  # Convertimos el ángulo de radianes a grados.
  angulo = angulo * 180 / np.pi

  return angulo


# Ejemplo
punto_recta = [1, 2, 3]
cuaternion_director = np.array([0, 0, 1, 1])

angulo = angulo_recta_plano(punto_recta, cuaternion_director)

print(angulo)







# import math

# pi=math.pi

# def calcular_angulo_alfa_y_eje(datos):
#     x = datos[0]
#     y = datos[1]
#     z = datos[2]
#     angulo_radianes_vertical = math.atan2(y,z)

#     angulo_radianes_vertical += pi/2

#     if angulo_radianes_vertical < 0:
#         angulo_radianes_vertical = 0
#     elif angulo_radianes_vertical > pi:
#         angulo_radianes_vertical = pi

#     angulo_grados_vertical = math.degrees(angulo_radianes_vertical)


#     print("--------------------------------------")
#     print("Posicion:" + str(datos))
#     print("Angulo vertical hombro:")
#     print(str(int(angulo_grados_vertical)) + "º \n")



# # Ejemplo de uso
# x = 1.0
# y = 1.0
# z = 1.0
# # calcular_angulo_alfa_y_eje((x, y, z))
# calcular_angulo_alfa_y_eje((x, y, z))