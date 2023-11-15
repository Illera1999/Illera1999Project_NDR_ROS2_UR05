import numpy as np

def calcular_angulo_entre_vectores(vector_a, vector_b):
    # producto_punto = np.dot(vector_a, vector_b)
    # magnitud_a = np.linalg.norm(vector_a)
    # print(f"Magnitud: {magnitud_a} \n")
    # magnitud_b = np.linalg.norm(vector_b)
    # print(f"Magnitud: {magnitud_b} \n")

    # coseno_theta = producto_punto / (magnitud_a * magnitud_b)
    # angulo_radianes = np.arccos(coseno_theta)
    
    producto_cross = np.cross(vector_a, vector_b)
    p_c = np.linalg.norm(producto_cross)
    angulo_radianes = np.arcsin(p_c)

    angulo_grados = np.degrees(angulo_radianes)

    return angulo_radianes, angulo_grados


# Ejemplo de uso
vector_a = np.array([-0.99132876, 0.05805142, -0.11788837])
vector_b = np.array([0.88943682, 0.28638172, 0.35621292])

angulo_rad, angulo_grados = calcular_angulo_entre_vectores(vector_a, vector_b)

print(f"Ángulo en radianes: {angulo_rad}")
print(f"Ángulo en grados: {angulo_grados}")


