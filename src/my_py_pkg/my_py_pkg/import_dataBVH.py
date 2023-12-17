#!/usr/bin/env python3
import rclpy
import time
import rtde_control
import numpy as np
from rclpy.node import Node
from my_cpp_interfaces.msg import DataRightBVH
from my_py_pkg import calculate_angle



class BoneData:
    def __init__(self, name, rotation, quaternio, position):
        self.RightArm = [name[0], rotation[0], rotation[1], rotation[2]]
        self.RightArmQuaternio = [quaternio[0], quaternio[1], quaternio[2], quaternio[3]]
        self.RightForeArm = [name[1], rotation[3], rotation[4], rotation[5], quaternio[4], quaternio[5], quaternio[6], quaternio[7]]
        self.RightForeArmQuaternio = [quaternio[4], quaternio[5], quaternio[6], quaternio[7]]
        self.RightHand = [name[2], rotation[6], rotation[7], rotation[8], quaternio[8], quaternio[9], quaternio[10], quaternio[11]]
        self.RightArmPosition = [position[0], position[1], position[2]]
        self.RightForeArmPosition = [position[3], position[4], position[5]]
        self.RightHandPosition = [position[6], position[7], position[8]]
        self.name = name
        self.base, self.hombro = calculate_angle.angle_for_arm(self.RightArm[1], self.RightArm[2], self.RightArm[3], self.RightArmPosition[0], self.RightForeArmPosition[0])
        self.antebrazo = calculate_angle.angle_for_forearm(self.RightForeArm[1], self.RightForeArm[2], self.RightForeArm[3])

    def printData(self):
        print("------------------------------")
        print("Datos del Brazo: ")
        print(str(self.RightArm))
        print("           Position del Brazo: ")
        print(str(self.RightArmPosition))
        print("           Quaternio del Brazo: ")
        print(str(self.RightArmQuaternio))
        print("Datos del Ante brazo: ")
        print(str(self.RightForeArm))
        print("           Position del Antebrazo: ")
        print(str(self.RightForeArmPosition))
        print("           Quaternio del Antebrazo: ")
        print(str(self.RightForeArmQuaternio))
        print("Datos del Mano: ")
        print(str(self.RightHand))
        print("           Position del Mano: ")
        print(str(self.RightHandPosition))
        print("------------------------------")



    def base_angle(self): # La base va de 180 a -90
        angulo = self.base
        print("Angulo base sin tratar " + str(angulo))
        if angulo >=180 :
            angulo = 180
        if angulo < -90:
            angulo = -90
        print("Angulo base " + str(angulo))
        angulo = np.radians(angulo)
        return angulo
    
    def hombro_angle(self): # El hombro recorre desde -0 a -90
        angulo = self.hombro
        print("Angulo hombro sin tratar" + str(angulo))
        if angulo > 0:
            angulo = 0
        if angulo < -90:
            angulo = -90
        print("Angulo hombro " + str(angulo))
        angulo = np.radians(angulo)
        return angulo
    
    def codo_angle(self): # El codo va de entre 0 a 90
        angulo,_,_,_,_ = calculate_angle.calculate_elbow_angle(self.RightArmQuaternio, self.RightArmPosition,
                                                    self.RightForeArmQuaternio, self.RightForeArmPosition)
        if angulo < 0:
            angulo = 0
        if angulo > 90:
            angulo = 90
        print("Angulo codo " + str(angulo))
        angulo = np.radians(-angulo)
        return angulo

    def muñeca1_angle(self): # Levantar la muñeca va entre -180 y -60 ( Siendo -60  para evitar golpes contra la mesa)
        angulo = self.RightHand[3]
        print("Angulo muñeca_1  sin tratar " + str(angulo))
        angulo -= 90
        if angulo < -180:
            angulo = -180
        if angulo > -60:
            angulo = -60
        print("Angulo muñeca_1 " + str(angulo))
        angulo = np.radians(angulo)
        return angulo
    
    def muñeca2_angle(self):
        angulo = self.RightHand[2] + self.RightForeArm[2] + self.RightArm[2]
        print("Angulo muñeca_2  sin tratar " + str(angulo))
        angulo -= 90
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
        rtde_c.moveJ([rightArm.base_angle(), rightArm.hombro_angle(), rightArm.codo_angle(),
                       rightArm.muñeca1_angle(), rightArm.muñeca2_angle(), rightArm.muñeca3_angle()], 1.5, 1.5)

rtde_c = rtde_control.RTDEControlInterface("192.168.0.100")


def main(args=None):
    rtde_c.moveJ([1.5708, 0, 0, -1.5708, -1.5708, 0], 1, 1)
    rclpy.init(args=args)
    node = ImportData() 
    rclpy.spin(node)
    rclpy.shutdown()
    rtde_c.stopJ()

    return 0

if __name__ == "__main__":
    main()

