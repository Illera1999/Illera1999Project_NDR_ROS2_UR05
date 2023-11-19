#!/usr/bin/env python3
import rclpy
import time
import rtde_control
import numpy as np
import transforms3d.quaternions as quaternions
from rclpy.node import Node
from my_cpp_interfaces.msg import DataRight

# rtde_c = rtde_control.RTDEControlInterface("192.168.0.100")
pi = np.pi

class BoneData:
    def __init__(self, name, position, quaternio):
        self.RightShoulder = [name[0], position[0], position[1], position[2], quaternio[0], quaternio[1], quaternio[2], quaternio[3]]
        self.RightArm = [name[1], position[3], position[4], position[5], quaternio[4], quaternio[5], quaternio[6], quaternio[7]]
        self.RightForeArm = [name[2], position[6], position[7],  position[8], quaternio[8], quaternio[9], quaternio[10], quaternio[11]]
        self.RightHand = [name[3], position[9], position[10], position[11], quaternio[12], quaternio[13], quaternio[14], quaternio[15]]
        self.name = name
        self.position = position    

    
    def get_position_quaternio_shoulder(self):
        return self.RightShoulder

    def get_position_quaternio_arm(self):
        return self.RightArm

    def get_position_quaternio_foreArm(self):
        return self.RightForeArm

    def get_position_quaternio_hand(self):
        return self.RightForeArm

    def printData(self):
        print("------------------------------")
        print("Datos del Hombro: ")
        print(str(self.RightShoulder))
        print("Datos del Brazo: ")
        print(str(self.RightArm))
        print("Datos del Ante brazo: ")
        print(str(self.RightForeArm))
        print("Datos del Mano: ")
        print(str(self.RightHand))
        print("------------------------------")

class ImportData(Node): 
    def __init__(self):
        super().__init__("import_data_node") 
        self.number_subscriber_ = self.create_subscription(
            DataRight, "data_right_arm", self.callback_data, 10)
        self.get_logger().info("ImportData has been started.")
        
    def callback_data(self,msg):
        rightArm = BoneData(msg.name, msg.position, msg.quaternio)
        rightArm.printData()
        base, hombro = calcular_angulo_base_hombro(rightArm)

        # rtde_c.moveJ([1.5708, hombro, -0, -1.5708, 1.5708, 0], 1.5 , 1.5)
        # rtde_c.moveJ([base, -1.5708, -0, -1.5708, 1.5708, 0], 1.5, 1.5)
        # rtde_c.moveJ([base, hombro, -0, -1.5708, 1.5708, 0], 2, 2)
        #rtde_c.moveJ([1.5708, -base_vertical, -0, -1.5708, 1.5708, 0], 1, 1)
        #rtde_c.moveJ([1.5708, 0, ante_brazo_horizontal, -1.5708, 1.5708, 0], 1, 1)

def calcular_angulo_base_hombro(right):
    datos = right.get_position_quaternio_arm()
    datos1 = right.get_position_quaternio_foreArm()
    print("---------------- Soy el brazo ----------------")
    x, y, z = cuaternion_a_matriz_transformacion(datos[4:])
    print("---------------- Soy el ante brazo ----------------")
    x1, y1, z1 = cuaternion_a_matriz_transformacion(datos1[4:])
    radianes, angulo = calcular_angulo_entre_vectores([1,0,0], y)
    return radianes, angulo

def cuaternion_a_matriz_transformacion(cuaternion):
    # Convierte el cuaternión a una matriz de rotación
    matriz_rotacion = quaternions.quat2mat(cuaternion)
    print("Matriz de rotación \n")
    print(matriz_rotacion)

    # Obtiene las columnas de la matriz de rotación
    columna_x = matriz_rotacion[:, 0]
    columna_y = matriz_rotacion[:, 1]
    columna_z = matriz_rotacion[:, 2]

    return columna_x, columna_y, columna_z

def calcular_angulo_entre_vectores(vector_a, vector_b):
    # Calcular el producto punto
    producto_punto = np.dot(vector_a, vector_b)

    # Calcular las magnitudes de los vectores
    magnitud_v = np.linalg.norm(vector_a)
    magnitud_w = np.linalg.norm(vector_b)

    # Calcular el coseno del ángulo
    coseno_theta = producto_punto / (magnitud_v * magnitud_w)

    # Calcular el ángulo en radianes
    angulo_radianes = np.arccos(coseno_theta)
    # producto_punto = np.dot(vector_a, vector_b)
    # angulo_radianes = np.arccos(producto_punto)

    angulo_grados = np.degrees(angulo_radianes)

    print("-------------")
    print(f"Ángulo en radianes: {angulo_radianes}")
    print(f"Ángulo en grados: {angulo_grados}")
    print(f"Producto punto: {producto_punto}")


    return angulo_radianes, angulo_grados

def main(args=None):
    # rtde_c.moveJ([1.5708, 0, 0, -1.5708, 1.5708, 0], 1, 1)
    time.sleep(2)
    rclpy.init(args=args)
    node = ImportData() 
    rclpy.spin(node)
    rclpy.shutdown()
    # rtde_c.stopJ()
    return 0

if __name__ == "__main__":
    main()

