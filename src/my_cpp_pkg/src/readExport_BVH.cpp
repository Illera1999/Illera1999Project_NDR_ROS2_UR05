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
    float dx, dy, dz;
    float qw, qx, qy, qz;

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
        std::cout << "Position: " << std::endl;
        std::cout << "{" << data.dx << ", " << data.dy << ", "<< data.dz << "}" << std::endl;
        std::cout << "Quaternios: " << std::endl;
        std::cout << "{" << data.qw << ", " << data.qx << ", "<< data.qy << ", " << data.qz << "}" << std::endl;
        std::cout << "\n" << std::endl;
    }

    std::cout << "\n" << std::endl; 
}

static void CalcDataFromHand(void* customedObj, SOCKET_REF sender, CalcDataHeader* header, float* data)
{
    int bone = 8;
    
    myMutex.lock();
    /*
    La variable BoneData lleva un "&"
    para indicar que no queremos que arm sea una copia
    del contenido de rightArm si no el obejto de verdad.
    */
    for(BoneData& arm: rightArm)
    {
        arm.dx = data[bone * 16 + 0];
        arm.dy = data[bone * 16 + 1];
        arm.dz = data[bone * 16 + 2];
        arm.qw = data[bone * 16 + 6];
        arm.qx = data[bone * 16 + 7];
        arm.qy = data[bone * 16 + 8];
        arm.qz = data[bone * 16 + 9];
        bone = bone + 1;
    }
    myMutex.unlock();

}

static void frameDataFromHand(void* customedObj, SOCKET_REF sender, BvhDataHeader* header, float* data)
{
    int aux = 0;
    int sds = 14;
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
        arm.rx = data[3 + (sds * 3)];
        arm.ry = data[3 + (sds * 3) + 1];
        arm.rz = data[3 + (sds * 3) + 2];
        sds = sds + 1;
        aux += 1;
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
        int auxiii = 0;
        myMutex.lock();
        for (const BoneData& data: rightArm){
            msg.name[auxi] = data.name;
            msg.rotation[auxii] = data.rx;
            msg.rotation[auxii + 1] = data.ry;
            msg.rotation[auxii + 2] = data.rz;
            msg.position[auxii] = data.dx;
            msg.position[auxii + 1] = data.dy;
            msg.position[auxii + 2] = data.dz;
            msg.quaternio[auxiii] = data.qw;
            msg.quaternio[auxiii + 1] = data.qx;
            msg.quaternio[auxiii + 2] = data.qy;
            msg.quaternio[auxiii + 3] = data.qz;
            auxi += 1;
            auxii += 3;
            auxiii += 4;
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

    BRRegisterCalculationDataCallback(nullptr, CalcDataFromHand);
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
    int port_bvh = 7001;  
    int port_calc = 7003;  

    /*
    BRConnectTo(serverIP, port);
        Crea un cliente TCP/UDP para conectarse al 
            servidor.
    */
    SOCKET_REF socketRef_bvh = BRConnectTo(serverIP, port_bvh);
    SOCKET_REF socketRef_calc = BRConnectTo(serverIP, port_calc);

    while(true)
    {
        if (socketRef_bvh != NULL && socketRef_calc != NULL) 
        {
            /*
            BRGetSocketStatus(socketRef);
                Revisa el estado del Socket.
                Despues se publica por pantalla.
            */
            SocketStatus ssStatus_bvh = BRGetSocketStatus(socketRef_bvh);
            SocketStatus ssStatus_calc = BRGetSocketStatus(socketRef_calc);
            switch (ssStatus_bvh)
            {
                case CS_Running:
                    std::cout << "Conectado  BVH\n";
                    break;
                case CS_Starting:
                    std::cout << "Iniciando ... \n";
                    break;
                case CS_OffWork:
                    std::cout << "OffLine \n";
                    break;
            } 
            switch (ssStatus_calc)
            {
                case CS_Running:
                    std::cout << "Conectado Calc_BVH\n";
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
    BRCloseSocket(socketRef_bvh);
    BRCloseSocket(socketRef_calc);
}