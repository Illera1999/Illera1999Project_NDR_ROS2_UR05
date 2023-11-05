#!/usr/bin/env python3
import rclpy
import time
import rtde_control
import math
from rclpy.node import Node
from my_cpp_interfaces.msg import DataRight

#rtde_c = rtde_control.RTDEControlInterface("192.168.0.100")

class BoneData:
    def __init__(self, name, position):
        #self.RightShoulder = [name[0], position[0], position[1], position[2]]
        self.RightArm = [name[1], position[3], position[4], position[5]]
        self.RightForeArm = [name[2], position[6], position[7],  position[8]]
        self.RightHandº = [name[3], position[9], position[10], position[11]]
        self.name = name
        self.position = position

    def printData(self):
        #self.printauxData(self.RightShoulder)    
        self.printauxData(self.RightArm, self.position, self.name)    
        self.printauxData(self.RightForeArm, self.position, self.name)    
        self.printauxData(self.RightHando, self.position, self.name)    

    def printauxData(self, datos, position, name):
        self.RightArmValue = [name[1], position[3] - position[0], position[4] - position[1], position[5] - position[2]]
        self.RightForeArmValue = [name[2], position[6] - position[0], position[7] - position[1], position[8] - position[2]]
        #self.RightForeArmValueValue = [self.RightForeArmValue[0], self.RightForeArmValue[1] - self.RightArmValue[1], self.RightForeArmValue[2] - self.RightArmValue[2], self.RightForeArmValue[3] - self.RightArmValue[3]]
        self.RightHandValue = [name[3], position[9] - position[0], position[10] - position[1], position[11] - position[2]]

        print("------------------------------")
        print("Datos del brazo: ")
        print("Nombre: " + datos[0])
        print("Posición: { " + 
            str(datos[1]) + ", " + 
            str(datos[2]) + ", " + 
            str(datos[3]) + " }")
        print("Posición: { " + 
            str(self.RightArmValue[1]) + ", " + 
            str(self.RightForeArm[2]) + ", " + 
            str(self.RightHandValue[3]) + " }")
        print("------------------------------")

class ImportData(Node): 
    def __init__(self):
        super().__init__("import_data_node") 
        self.number_subscriber_ = self.create_subscription(
            DataRight, "data_right_arm", self.callback_data, 10)
        self.get_logger().info("ImportData has been started.")
        
    def callback_data(self,msg):
        rightArm = BoneData(msg.name, msg.position)
        rightArm.printData()
        #base_vertical, base_horizontal = calcular_angulo_base(rightArm.RightArmValue, vertical=False, horizontal=False)
        #ante_brazo_vertical, ante_brazo_horizontal = calcular_angulo_base(rightArm.RightForeArmValue, vertical=True, horizontal=True)
        #rtde_c.moveJ([-base_horizontal, -base_vertical, -0, -1.5708, 1.5708, 0], 1, 1)
        #rtde_c.moveJ([-base_horizontal, -base_vertical, -0, -1.5708, 1.5708, 0], 1, 1)
        #rtde_c.moveJ([1.5708, -base_vertical, -0, -1.5708, 1.5708, 0], 1, 1)
        #rtde_c.moveJ([1.5708, 0, ante_brazo_horizontal, -1.5708, 1.5708, 0], 1, 1)

# def cal_matriz_translate():
#     return 0

def calcular_angulo_base(datos, vertical = False, horizontal = False):
    x = datos[1]
    y = datos[2]
    z = datos[3]
    # Calcular el ángulo vertical en radianes con respecto al eje Z
    angulo_vertical_radianes = math.atan2(math.sqrt(x**2 + y**2), z)
    #angulo_horizontal_radianes = math.atan2(math.sqrt(x**2 + z**2), y)
    angulo_horizontal_radianes = math.atan2(y,x)

    # Convertir el ángulo de radianes a grados
    angulo_vertical_grados = math.degrees(angulo_vertical_radianes)
    # Convertir el ángulo horizontal de radianes a grados
    angulo_horizontal_grados = math.degrees(angulo_horizontal_radianes)

    if horizontal == True:
        print("Angulo de la base_horizontal")
        print(f"Ángulo horizontal en radianes: {angulo_horizontal_radianes}")
        print(f"Ángulo horizontal en grados: {angulo_horizontal_grados}")
    if vertical == True:
        print("Angulo de la base_vertical")
        print(f"Ángulo vertical en radianes: {angulo_vertical_radianes}")
        print(f"Ángulo vertical en grados: {angulo_vertical_grados}")
    return angulo_vertical_radianes, angulo_horizontal_radianes

def main(args=None):
    #rtde_c.moveJ([1.5708, 0, 0, -1.5708, 1.5708, 0], 1, 1)
    time.sleep(2)
    rclpy.init(args=args)
    node = ImportData() 
    rclpy.spin(node)
    rclpy.shutdown()
    rtde_c.stopJ()



if __name__ == "__main__":
    main()

