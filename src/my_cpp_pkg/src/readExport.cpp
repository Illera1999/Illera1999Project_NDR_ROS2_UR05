#include "rclcpp/rclcpp.hpp"
#include "my_cpp_interfaces/msg/data_right.hpp"
#include "NeuronDataReader.h"
#include "DataType.h"
#include "iostream"
#include "windows.h"
#include "string"
#include "vector"


struct BoneData 
{
    std::string name;
    float dx, dy, dz;
    float rx, ry, rz;

};
/*
Número del sensor que quieres coger datos:
*/
std::array<BoneData, 3> rightArm;
/*

std::ostream& operator<<(std::ostream& os, const BoneData& bone)
{
    os << "Nombre: " << bone.name << std::endl;
    os << "Desplazamiento (dx, dy, dz): " << bone.dx << ", " << bone.dy << ", " << bone.dz << std::endl;
    os << "Rotación (rx, ry, rz): " << bone.rx << ", " << bone.ry << ", " << bone.rz << std::endl;
    return os;
}
*/

int count = 0;
static void bvhFrameDataFromHand(void* customedObj, SOCKET_REF sender, BvhDataHeader* header, float* data)
{
    /*
    Guardamos datos:
        Variable global rightArm
        aux (int) por cada bucle i++ para guardar
        cada dato de los indices 14, 15, 16.
    */
    int bone = 8;


    int aux = 0;
    std::string name[] = {"RightArm", "RightForeArm", "RightHand"};
    for(BoneData arm: rightArm)
    {
        /*Index*/
        int index = (bone  + aux) * 16;
        if(header->WithReference)
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
        //std::cout << arm << std::endl;
    }
}

class ExportDataAxisNeuron : public rclcpp::Node
{
public:
    ExportDataAxisNeuron() : Node("export_data_axisneuron")
    {
        pub_ = this->create_publisher<my_cpp_interfaces::msg::DataRight>(
            "data_right_arm", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ExportDataAxisNeuron::publishDataAxisNeuron, this));
        RCLCPP_INFO(this->get_logger(), " ReadExport publisher has been started");
    }

private:
    void publishDataAxisNeuron()
    {
        auto msg = my_cpp_interfaces::msg::DataRight();
        int auxi = 0;
        int auxii = 0;
        for (BoneData data: rightArm){
            msg.name[auxi] = data.name;
            msg.desplazamiento[auxii] = data.dx;
            msg.desplazamiento[auxii + 1] = data.dy;
            msg.desplazamiento[auxii + 2] = data.dz;
            msg.rotacion[auxii] = data.dx;
            msg.rotacion[auxii + 1] = data.dy;
            msg.rotacion[auxii + 2] = data.dz;
            auxii += 3;
            auxi ++;
        }
        std::cout << "Datos del brazo: " << std::endl;
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
        std::cout << "\n" << std::endl;
        pub_->publish(msg);
    }

    rclcpp::Publisher<my_cpp_interfaces::msg::DataRight>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

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
            rclcpp::init(argc, argv);
            auto node = std::make_shared<ExportDataAxisNeuron>();
            rclcpp::spin(node);
            rclcpp::shutdown();
            
        } else 
        {
            std::cerr << "No se pudo establecer la conexion." << std::endl;
        }
        Sleep(500);
    }
    BRCloseSocket(socketRef);
}