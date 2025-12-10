#include "rclcpp/rclcpp.hpp"
#include "service_interfaces/srv/setangle.hpp"
#include "service_interfaces/srv/getangleact.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

//创建一个类节点，名字叫做Poorer,继承自Node.
class Poorer : public rclcpp::Node
{
public:
    // 构造函数
    Poorer() : Node("Poorer")
    {
        // 打印一句自我介绍
        RCLCPP_INFO(this->get_logger(), "实例化客户端");
        // 实例化客户端, 指明客户端的接口类型，同时指定要请求的服务的名称Calculate.
        Poorer_Client = this->create_client<service_interfaces::srv::Getangleact>("Getangleact");
    }

    int take_hamburger(int argc, char **argv)
    {
        RCLCPP_INFO(this->get_logger(), "现在读取灵巧手实际角度");
        
        //构造请求
        auto request = std::make_shared<service_interfaces::srv::Getangleact::Request>();
        
        //等待服务端上线
        while (!Poorer_Client->wait_for_service(1s))
        {
            //等待时检测rclcpp的状态
            if (!rclcpp::ok())
            {
                // 检测到Ctrl+C直接退出
                RCLCPP_ERROR(this->get_logger(), "等待被打断, 不等了");
                rclcpp::shutdown();
                return 1;
            }
            // 否则一直等
            RCLCPP_INFO(this->get_logger(), "等待读取");
        }

        // 输入参数格式错误的时候报错并退出程序
        if (argc != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "输入格式错误, 格式为: 灵巧手指令，灵巧手ID");
            rclcpp::shutdown();
            return 1;
        }
        else
        {
            // 格式正确, 获取参数, 放入request中
            request->status =             argv[1];
            request->hand_id = atoi(argv[2]);
            RCLCPP_INFO(this->get_logger(), "指令为%s, 灵巧手ID为%d", request->status.c_str(), request->hand_id);
        }

        //发送异步请求，然后等待返回，返回时调用回调函数
        Poorer_Client->async_send_request(request, std::bind(&Poorer::poorer_callback, this, _1));

        return 0;
    }
private:
    // 创建一个客户端
    rclcpp::Client<service_interfaces::srv::Getangleact>::SharedPtr Poorer_Client;

    // 创建接收到小说的回调函数
    void poorer_callback(rclcpp::Client<service_interfaces::srv::Getangleact>::SharedFuture response)
    {
        // 使用response的get()获取
        auto result = response.get();
        // 如果确实是Poorer, 则领取成功
        RCLCPP_INFO(this->get_logger(), "灵巧手实际角度为:%d %d %d %d %d %d", 
                    result->curangleact[0],result->curangleact[1],result->curangleact[2],result->curangleact[3],result->curangleact[4],result->curangleact[5]);
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 产生一个Poorer的节点
    auto node = std::make_shared<Poorer>();
    node->take_hamburger(argc, argv);
    // 运行节点，并检测rclcpp状态
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

