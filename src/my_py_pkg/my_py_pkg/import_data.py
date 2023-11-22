#!/usr/bin/env python3
import rclpy
import time
import rtde_control
import numpy as np
from my_py_pkg import calculate_angle
import transforms3d.quaternions as quaternions
from rclpy.node import Node
from my_cpp_interfaces.msg import DataRight

rtde_c = rtde_control.RTDEControlInterface("192.168.0.100")

class BoneData:
    def __init__(self, name, position, quaternio):
        self.RightShoulder = [name[0], position[0], position[1], position[2], quaternio[0], quaternio[1], quaternio[2], quaternio[3]]
        self.RightArm = [name[1], position[3], position[4], position[5], quaternio[4], quaternio[5], quaternio[6], quaternio[7]]
        self.RightForeArm = [name[2], position[6], position[7],  position[8], quaternio[8], quaternio[9], quaternio[10], quaternio[11]]
        self.RightHand = [name[3], position[9], position[10], position[11], quaternio[12], quaternio[13], quaternio[14], quaternio[15]]
        self.name = name
        self.position = position    

    def two_points_of_arm(self):
        puntos = [[dato[1], dato[2], dato[3]] for dato in [self.RightShoulder, self.RightArm]]

        primer_dato = puntos[0]
        puntos = [[dato[0] - primer_dato[0], dato[1] - primer_dato[1], dato[2] - primer_dato[2]] for dato in puntos]
        return puntos[1]
    
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
        base = calculate_angle.base_angel(rightArm.two_points_of_arm())
        hombro = calculate_angle.hombro_angle(rightArm.two_points_of_arm())
        rtde_c.moveJ([1.5708 - hombro, -base, 0, -1.5708, 1.5708, 0], 1.5, 1.5)

def main(args=None):
    rtde_c.moveJ([1.5708, 0, 0, -1.5708, 1.5708, 0], 1, 1)
    time.sleep(2)
    rclpy.init(args=args)
    node = ImportData() 
    rclpy.spin(node)
    rclpy.shutdown()
    # rtde_c.stopJ()
    return 0

if __name__ == "__main__":
    main()

