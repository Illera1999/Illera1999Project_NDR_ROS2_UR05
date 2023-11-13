#!/usr/bin/env python3
import rclpy
import time
import rtde_control
import math
from rclpy.node import Node
from my_cpp_interfaces.msg import DataRight

rtde_c = rtde_control.RTDEControlInterface("192.168.0.100")
pi = math.pi

class BoneData:
    def __init__(self, name, position, quaternio):
        self.RightShoulder = [name[0], position[0], position[1], position[2], quaternio[0], quaternio[1], quaternio[2], quaternio[3]]
        self.RightShoulderVakue = [name[0], position[0] - position[0], position[1] - position[1], position[2] - position[2]]

        self.RightArm = [name[1], position[3], position[4], position[5], quaternio[4], quaternio[5], quaternio[6], quaternio[7]]
        self.RightArmValue = [name[1], position[3] - position[0], position[4] - position[1], position[5] - position[2], self.RightArm[4]]
        self.RightForeArm = [name[2], position[6], position[7],  position[8], quaternio[8], quaternio[9], quaternio[10], quaternio[11]]
        self.RightHand = [name[3], position[9], position[10], position[11], quaternio[12], quaternio[13], quaternio[14], quaternio[15]]
        self.name = name
        self.position = position        

    def printData(self):
        print("------------------------------")
        print("Datos del brazo: ")

        print(str(self.RightArm))
        print(str(self.RightArmValue))

        # print(str(self.RightArmValue))
        # print(str(self.RightForeArm))
        # print(str(self.RightHand))
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
        base, hombro = calcular_angulo_base_hombro(rightArm.RightArmValue)

        # rtde_c.moveJ([1.5708, hombro, -0, -1.5708, 1.5708, 0], 1.5 , 1.5)
        # rtde_c.moveJ([base, -1.5708, -0, -1.5708, 1.5708, 0], 1.5, 1.5)
        # rtde_c.moveJ([base, hombro, -0, -1.5708, 1.5708, 0], 2, 2)
        #rtde_c.moveJ([1.5708, -base_vertical, -0, -1.5708, 1.5708, 0], 1, 1)
        #rtde_c.moveJ([1.5708, 0, ante_brazo_horizontal, -1.5708, 1.5708, 0], 1, 1)

def calcular_angulo_base_hombro(datos):
    x = datos[1]
    y = datos[2]
    z = datos[3]
    quaternio = datos[4]

    # -----------------------     Calcula el ángulo entre el vector y el eje x,z usando atan2 ---------------------------------
    angulo_radianes_horizontal = 0

    # -----------------------     Calcula el angulo formado por el plano y,z ---------------------------------

    angulo_radianes_vertical = math.atan2(y,z)

    if angulo_radianes_vertical > 0 and quaternio <= 0:
        angulo_radianes_vertical = 0
    if angulo_radianes_vertical > 0 and quaternio > 0:
        angulo_radianes_vertical = -pi
    
    
    angulo_grados_vertical = math.degrees(angulo_radianes_vertical)


    print("--------------------------------------")
    print("Posicion:" + str(datos))
    print("Angulo vertical hombro:")
    print(str(int(angulo_grados_vertical)) + "º \n")
    # print("Angulo horizontal hombro:")
    # print(str(int(angulo_grados_horizontal)) + "º \n")

    return angulo_radianes_horizontal, angulo_radianes_vertical

def main(args=None):
    rtde_c.moveJ([1.5708, 0, 0, -1.5708, 1.5708, 0], 1, 1)
    time.sleep(2)
    rclpy.init(args=args)
    node = ImportData() 
    rclpy.spin(node)
    rclpy.shutdown()
    rtde_c.stopJ()
    return 0

if __name__ == "__main__":
    main()

