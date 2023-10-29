#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_cpp_interfaces.msg import DataRight

class BoneData:
    def __init__(self):
        self.array_name = []
        self.array_desplazamiento = []
        self.array_rotacion = []
    
    def printData(self):
        print("Datos del brazo: ")
        aux = 0
        for name in self.array_name:
            print("Nombre: " + name)
            print("Desplazamiento: { " + 
                str(self.array_desplazamiento[aux]) + ", " + 
                str(self.array_desplazamiento[aux + 1]) + ", " + 
                str(self.array_desplazamiento[aux + 2]) + " }")
            print("Rotación: { " + 
                str(self.array_rotacion[aux]) + ", " + 
                str(self.array_rotacion[aux + 1]) + ", " + 
                str(self.array_rotacion[aux + 2]) + " }")
            aux += 3 


class ImportData(Node): 
    def __init__(self):
        super().__init__("import_data_node") 
        self.number_subscriber_ = self.create_subscription(
            DataRight, "data_right_arm", self.callback_data, 10)
        self.get_logger().info("ImportData has been started.")
        
    def callback_data(self,msg):
        rightArm = BoneData()
        rightArm.array_name = msg.name
        rightArm.array_desplazamiento = msg.desplazamiento
        rightArm.array_rotacion = msg.rotacion
        rightArm.printData()

    


def main(args=None):
    rclpy.init(args=args)
    node = ImportData() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

