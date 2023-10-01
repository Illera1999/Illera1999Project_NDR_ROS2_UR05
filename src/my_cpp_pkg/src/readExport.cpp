#include "rclcpp/rclcpp.hpp"
#include "NeuronDataReader.h"
#include "DataType.h"
#include "iostream"
#include "windows.h"
#include "string"

struct BoneData 
{
    std::string name;
    float dx, dy, dz;
    float rx, ry, rz;

};
/*
std::ostream& operator<<(std::ostream& os, const BoneData& bone)
{
    os << "Nombre: " << bone.name << std::endl;
    os << "Desplazamiento (dx, dy, dz): " << bone.dx << ", " << bone.dy << ", " << bone.dz << std::endl;
    os << "Rotación (rx, ry, rz): " << bone.rx << ", " << bone.ry << ", " << bone.rz << std::endl;
    return os;
}
*/

static void bvhFrameDataFromHand(void* customedObj, SOCKET_REF sender, BvhDataHeader* header, float* data)
{

    std::cout << "Datos del brazo: " << std::endl;

    std::array<BoneData, 3> rightArm;
    /*
    Número del sensor que quieres coger datos:
        8 -> RightArm
        9 -> RightForeArm
        10 -> RightHand
    */
    int bone = 8;

    /*
    Guardamos datos:
        Variable global rightArm
        aux (int) por cada bucle i++ para guardar
        cada dato de los indices 14, 15, 16.
    */
    int aux = 0;
    std::string name[] = {"RightArm", "RightForeArm", "RightHand"};
    for(BoneData arm: rightArm)
    {
        /*Index*/
        int index = (bone  + aux) * 6;
        if(header->WithDisp)
        {
            index += 6;
        }
        arm.name = name[aux];
        arm.dx = data[index + 0];
        arm.dy = data[index + 1];
        arm.dz = data[index + 2];
        arm.rx = data[index + 3];
        arm.ry = data[index + 4];
        arm.rz = data[index + 5];
        aux ++;
        char strBuff[32];
        std::cout << "Nombre: " << arm.name << std::endl;
        sprintf_s(strBuff, sizeof(strBuff), "%0.3f", arm.dx);
        std::cout << "X = {" << strBuff;
        sprintf_s(strBuff, sizeof(strBuff), "%0.3f", arm.rx);
        std::cout << ", " << strBuff << "} ";
        sprintf_s(strBuff, sizeof(strBuff), "%0.3f", arm.dy);
        std::cout << "Y = {" << strBuff;
        sprintf_s(strBuff, sizeof(strBuff), "%0.3f", arm.ry);
        std::cout << ", " << strBuff << "} ";
        sprintf_s(strBuff, sizeof(strBuff), "%0.3f", arm.dz);
        std::cout << "Z = {" << strBuff;
        sprintf_s(strBuff, sizeof(strBuff), "%0.3f", arm.rz);
        std::cout << ", " << strBuff << "}" << std::endl;
        //std::cout << arm << std::endl;
    }
    std::cout << "\n" << std::endl;
}


int main(int argc, char **argv)
{
    /*
    BRRegisterFrameDataCallback(this, bvhFrameDataFromHand);
        Método callback para la recopilación de datos.
    */
    BRRegisterFrameDataCallback(nullptr, bvhFrameDataFromHand);

    /*
    Variables serverIP y port:
        serverIP (char): Dirección IP al servidor de 
            AxisNeuron
            Como Axis Neuron se ejecuta en el propio
            ordenador la IP = 127.0.0.1
        port (int): Puerto por el que conectarse a AxisNeuron
            8012 -> Data Stream Port
            7001 -> BVH Data
    */
    char serverIP[] = "127.0.0.1";
    int port = 7001; 

    /*
    BRConnectTo(serverIP, port);
        Crea un cliente TCP/UDP para conectarse al 
            servidor.
    */
    SOCKET_REF socketRef = BRConnectTo(serverIP, port);

    while(true)
    {
        if (socketRef != NULL) 
        {
            /*
            BRGetSocketStatus(socketRef);
                Revisa el estado del Socket.
                Despues se publica por pantalla.
            */
            SocketStatus ssStatus = BRGetSocketStatus(socketRef);
            switch (ssStatus)
            {
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
        } else 
        {
            std::cerr << "No se pudo establecer la conexion." << std::endl;
        }
        Sleep(500);
    }
    BRCloseSocket(socketRef);
}