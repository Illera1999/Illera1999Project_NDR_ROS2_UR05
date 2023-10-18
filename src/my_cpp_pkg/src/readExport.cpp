#include "rclcpp/rclcpp.hpp"
#include "my_cpp_interfaces/msg/data_right.hpp"
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
    float dx, dy, dz;
    float rx, ry, rz;

};
std::array<BoneData, 3> rightArm;

/*
Método para imprimir los datos del brazo.
*/
void printData()
{
    std::cout << "Datos del brazo: " << std::endl;
    for(BoneData data: rightArm){
        std::cout << "Nombre: " << data.name << std::endl;
        std::cout << "Desplazamiento: " << std::endl;
        std::cout << "{" << data.dx << ", " << data.dy << ", "<< data.dz << "}" << std::endl;
        std::cout << "Rotación : " << std::endl;
        std::cout << "{" << data.rx << ", " << data.ry << ", "<< data.rz << "}" << std::endl;
        std::cout << "\n" << std::endl;
    }

    std::cout << "\n" << std::endl; 
}

void printCalcData()
{
    std::cout << "Datos del brazo: " << std::endl;
    for(BoneData data: rightArm){
        std::cout << "Nombre: " << data.name << std::endl;
        std::cout << "Posición: " << std::endl;
        std::cout << "{" << data.dx << ", " << data.dy << ", "<< data.dz << "}" << std::endl;
        std::cout << "Velocidad : " << std::endl;
        std::cout << "{" << data.rx << ", " << data.ry << ", "<< data.rz << "}" << std::endl;
        std::cout << "\n" << std::endl;
    }

    std::cout << "\n" << std::endl; 
}

static void bvhFrameDataFromHand(void* customedObj, SOCKET_REF sender, BvhDataHeader* header, float* data)
{

    /*
    Número del sensor que quieres coger datos:
        8 -> RightArm
        9 -> RightForeArm
        10 -> RightHand
    */
    int bone = 8;

    /*
    Datos neceasrios para pasar correctamente los datos.
    */
    int aux = 0;
    std::string name[] = {"RightArm", "RightForeArm", "RightHand"};

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
        /*Index*/
        int index = (bone + aux) * 6;
        arm.name = name[aux];
        arm.dx = data[index + 0];
        arm.dy = data[index + 1];
        arm.dz = data[index + 2];
        arm.rx = data[index + 3];
        arm.ry = data[index + 4];
        arm.rz = data[index + 5];
        aux ++;
    }
    myMutex.unlock();
}

static void calculationDataFromHand(void* customedObj, SOCKET_REF sender, CalcDataHeader* header, float* data)
{

    /*
    Número del sensor que quieres coger datos:
        8 -> RightArm
        9 -> RightForeArm
        10 -> RightHand
    */
    int bone = 8;

    /*
    Datos neceasrios para pasar correctamente los datos.
    */
    int aux = 0;
    std::string name[] = {"RightArm", "RightForeArm", "RightHand"};

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
        /*Index*/
        int index = (bone + aux) * 16;
        arm.name = name[aux];
        arm.dx = data[index + 0];
        arm.dy = data[index + 1];
        arm.dz = data[index + 2];
        arm.rx = data[index + 10];
        arm.ry = data[index + 11];
        arm.rz = data[index + 12];
        aux ++;
    }
    myMutex.unlock();
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
        myMutex.lock();
        for (BoneData data: rightArm){
            msg.name[auxi] = data.name;
            msg.desplazamiento[auxii] = data.dx;
            msg.desplazamiento[auxii + 1] = data.dy;
            msg.desplazamiento[auxii + 2] = data.dz;
            msg.rotacion[auxii] = data.rx;
            msg.rotacion[auxii + 1] = data.ry;
            msg.rotacion[auxii + 2] = data.rz;
            auxii += 3;
            auxi ++;
        }
        printCalcData();
        myMutex.unlock();
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
    //BRRegisterFrameDataCallback(nullptr, bvhFrameDataFromHand);
    BRRegisterCalculationDataCallback(nullptr, calculationDataFromHand);

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
    //int port = 7001;  
    int port = 7003;  

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