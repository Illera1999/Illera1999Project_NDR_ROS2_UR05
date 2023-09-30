#include "rclcpp/rclcpp.hpp"
#include "NeuronDataReader.h"
#include "DataType.h"
#include "iostream"

struct BoneData {
    std::string name;
    float x, y, z;
};

std::array<BoneData, 3> rightArm;

int main(int argc, char **argv)
{
    /*
    Variables serverIP y port:
        serverIP (char): Dirección IP al servidor de 
            AxisNeuron
        port (int): Puerto por el que conectarse a AxisNeuron
            7009 -> General
            8012 -> Data Stream Port
            7001 -> BVH
            7003 -> Calculation
    */
    char serverIP[] = "192.168.0.101";
    int port = 7003; 

    /*
    BRConnectTo(serverIP, port);
        Crea un cliente TCP/UDP para conectarse al 
            servidor.
    */
    SOCKET_REF socketRef = BRConnectTo(serverIP, port);
    while(true){
        if (socketRef != NULL) {
            /*
            BRGetSocketStatus(socketRef);
                Revisa el estado del Socket.
                Despues se publica por pantalla.
            */
            SocketStatus ssStatus = BRGetSocketStatus(socketRef);
            switch (ssStatus){
                case CS_Running:
                    std::cout << "Conectado \n";
                    break;
                case CS_Starting:
                    std::cout << "Iniciando ... \n";
                    break;
                case CS_OffWork:
                    std::cout << "OffLine \n";
                    break;
            }
        } else {
            std::cerr << "No se pudo establecer la conexion." << std::endl;
        }
    }
    BRCloseSocket(socketRef);
    return 0;

}