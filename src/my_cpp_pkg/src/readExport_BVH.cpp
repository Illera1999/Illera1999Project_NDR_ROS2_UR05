#include "rclcpp/rclcpp.hpp"
#include "my_cpp_interfaces/msg/data_right_bvh.hpp"
#include "NeuronDataReader.h"
#include "DataType.h"
#include "iostream"
#include "windows.h"
#include "string"
#include "vector"
#include "mutex"

std::mutex myMutex;

struct BoneData 
{
    std::string name;
    float rx, ry, rz;

};

std::array<BoneData, 3> rightArm;
std::string nameBone[] = { "RightArm", "RightForeArm", "RightHand"};
/*
Número del sensor que quieres coger datos:
    45 -> RightArm
*/

/*
Método para imprimir los datos del brazo.
*/

void printCalcData()
{
    std::cout << "Datos del brazo: " << std::endl;
    for(BoneData data: rightArm){
        std::cout << "Nombre: " << data.name << std::endl;
        std::cout << "Rotaciones: " << std::endl;
        std::cout << "{" << data.rx << ", " << data.ry << ", "<< data.rz << "}" << std::endl;
        std::cout << "\n" << std::endl;
    }

    std::cout << "\n" << std::endl; 
}

static void frameDataFromHand(void* customedObj, SOCKET_REF sender, BvhDataHeader* header, float* data)
{
    // for (int i = 0; i < 100; ++i){
    //     if (data[i] > 94){
    //         std::cout << "{ ------------------------------- }" << std::endl;
    //         std::cout << "{" << data[i] << " - posicion " << i << "}" << std::endl;
    //     }
    // }
    /*
    Datos neceasrios para pasar correctamente los datos.
    */
    int aux = 0;
    int auxi = 0;
    /*
    Creo un Mutex.
    */
    myMutex.lock();
    /*
    La variable BoneData lleva un "&"
    para indicar que no queremos que arm sea una copia
    del contenido de rightArm si no el obejto de verdad.
    */
    for(BoneData& arm: rightArm)
    {
        arm.name = nameBone[aux];
        arm.rx = data[45 + auxi];
        arm.ry = data[46 + auxi];
        arm.rz = data[47 + auxi];
        aux = aux + 1;
        auxi = auxi +3;
    }
    myMutex.unlock();
}

class ExportDataAxisNeuron : public rclcpp::Node
{
public:
    ExportDataAxisNeuron() : Node("export_data_axisneuron")
    {
        pub_ = this->create_publisher<my_cpp_interfaces::msg::DataRightBVH>(
            "data_right_arm", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ExportDataAxisNeuron::publishDataAxisNeuron, this));
        RCLCPP_INFO(this->get_logger(), " ReadExport publisher has been started");
    }

private:
    void publishDataAxisNeuron()
    {
        auto msg = my_cpp_interfaces::msg::DataRightBVH();
        int auxi = 0;
        int auxii = 0;
        myMutex.lock();
        for (const BoneData& data: rightArm){
            msg.name[auxi] = data.name;
            msg.rotation[auxii] = data.rx;
            msg.rotation[auxii + 1] = data.ry;
            msg.rotation[auxii + 2] = data.rz;
            auxi += 1;
            auxii += 3;
        }
        printCalcData();
        myMutex.unlock();
        pub_->publish(msg);
    }

    rclcpp::Publisher<my_cpp_interfaces::msg::DataRightBVH>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    /*
    BRRegisterFrameDataCallback(this, bvhFrameDataFromHand);
        Método callback para la recopilación de datos.
    */
    BRRegisterFrameDataCallback(nullptr, frameDataFromHand);

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
    // int port = 7003;  

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