#!/usr/bin/env python3
import rclpy
import time
import rtde_control
import numpy as np
from rclpy.node import Node
from my_cpp_interfaces.msg import DataRightBVH
from my_py_pkg import combination_angle


last_angle = True

class BoneData:
    def __init__(self, name, rotation, quaternio, position):
        self.RightArm = [name[0], rotation[0], rotation[1], rotation[2], quaternio[0], quaternio[1], quaternio[2], quaternio[3]]
        self.RightForeArm = [name[1], rotation[3], rotation[4], rotation[5], quaternio[4], quaternio[5], quaternio[6], quaternio[7]]
        self.RightHand = [name[2], rotation[6], rotation[7], rotation[8], quaternio[8], quaternio[9], quaternio[10], quaternio[11]]
        self.RightArmPosition = [position[0], position[1], position[2]]
        self.RightForeArmPosition = [position[3], position[4], position[5]]
        self.RightHandPosition = [position[6], position[7], position[8]]
        self.name = name

    def printData(self):
        print("------------------------------")
        print("Datos del Brazo: ")
        print(str(self.RightArm))
        print("           Position del Brazo: ")
        print(str(self.RightArmPosition))
        print("Datos del Ante brazo: ")
        print(str(self.RightForeArm))
        print("           Position del AnteBrazo: ")
        print(str(self.RightForeArmPosition))
        print("Datos del Mano: ")
        print(str(self.RightHand))
        print("           Position del Mano: ")
        print(str(self.RightHandPosition))
        print("------------------------------")
    
    def hombro_angle(self): #El hombro recorre desde -0 a -90
        angulo = self.RightArm[3] # El valor z es el correspondiente a la hombro
        if angulo > 0:
            angulo = 0
        if angulo < -90:
            angulo = -90
        print("Angulo hombro " + str(angulo))
        angulo = np.radians(angulo)
        return angulo
    
    def base_angle(self):
        global last_angle
        angulo = self.RightArm[1] # El valor y es el correspondiente a la base
        if -180 < angulo < -90 and last_angle == True:
            angulo = 180 + angulo
            angulo += 180
            last_angle = True
            if angulo > 270:
                angulo = 270
        elif -180 < angulo < -90 and last_angle == False:
            last_angle = False
            angulo = -90
        if angulo > 0:
            last_angle = True
        else:
            last_angle = False
        print("Angulo base " + str(angulo))
        angulo = np.radians(angulo)
        return angulo
    
    def codo_angle(self):
        angulo = self.RightForeArm[1]
        if angulo < 0:
            angulo = 0
        if angulo > 90:
            angulo = 90
        print("Angulo codo " + str(angulo))
        angulo = np.radians(-angulo)
        return angulo

    def muñeca1_angle(self):
        angulo = self.RightHand[3]
        angulo -= 90
        if angulo < -180:
            angulo = -180
        if angulo > -80:
            angulo = -80
        print("Angulo muñeca_1 " + str(angulo))
        angulo = np.radians(angulo)
        return angulo
    
    def muñeca2_angle(self):
        angulo = self.RightForeArm[2]
        angulo = -angulo
        angulo -= 50
        if angulo > 0:
            angulo = 0
        if angulo < -180:
            angulo = -180

        print("Angulo muñeca_2 " + str(angulo))
        angulo = np.radians(angulo)
        return angulo
    
    def muñeca3_angle(self):
        angulo = self.RightHand[1]
        print("Angulo muñeca_3 " + str(angulo))
        angulo = np.radians(angulo)
        return angulo
    
class ImportData(Node): 
    def __init__(self):
        super().__init__("import_data_node") 
        self.number_subscriber_ = self.create_subscription(
            DataRightBVH, "data_right_arm", self.callback_data, 10)
        self.get_logger().info("ImportData has been started.")
        
    def callback_data(self,msg):
        rightArm = BoneData(msg.name, msg.rotation, msg.quaternio, msg. position)
        rightArm.printData()

        # rtde_c.moveJ([ 1.5708, rightArm.hombro_angle(), 0, -1.5708, -1.5708, 0], 1.5, 1.5)
        # rtde_c.moveJ([ rightArm.base_angle(), 0, 0, -1.5708, -1.5708, 0], 1.5, 1.5)
        # rtde_c.moveJ([1.5708, 0, rightArm.codo_angle(), -1.5708, -1.5708, 0], 1.5, 1.5)
        # rtde_c.moveJ([rightArm.base_angle(), rightArm.hombro_angle(), rightArm.codo_angle(), -1.5708, -1.5708, 0], 1.5, 1.5)
        # rtde_c.moveJ([1.5708, 0, codo, -1.5708, 1.5708, 0], 1.5, 1.5)
        # rtde_c.moveJ([rightArm.base_angle(), rightArm.hombro_angle(), rightArm.codo_angle(),
        #                rightArm.muñeca1_angle(), rightArm.muñeca2_angle(), rightArm.muñeca3_angle()], 1.5, 1.5)

# rtde_c = rtde_control.RTDEControlInterface("192.168.0.100")


def main(args=None):
    # rtde_c.moveJ([1.5708, 0, 0, -1.5708, -1.5708, 0], 1, 1)
    rclpy.init(args=args)
    node = ImportData() 
    rclpy.spin(node)
    rclpy.shutdown()
    # rtde_c.stopJ()

    return 0

if __name__ == "__main__":
    main()

