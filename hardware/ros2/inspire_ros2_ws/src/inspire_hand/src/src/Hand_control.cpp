#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "service_interfaces/srv/setangle.hpp"
#include "service_interfaces/srv/getangleact.hpp"
#include "service_interfaces/srv/setpos.hpp"
#include "service_interfaces/srv/setspeed.hpp"
#include "service_interfaces/srv/setforce.hpp"
#include "service_interfaces/srv/setgestureno.hpp"
#include "service_interfaces/srv/getangleset.hpp"
#include "service_interfaces/srv/getposact.hpp"
#include "service_interfaces/srv/getposset.hpp"
#include "service_interfaces/srv/getspeedset.hpp"
#include "service_interfaces/srv/getforceact.hpp"
#include "service_interfaces/srv/getforceset.hpp"
#include "service_interfaces/srv/getcurrentact.hpp"
#include "service_interfaces/srv/geterror.hpp"
#include "service_interfaces/srv/gettemp.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[64] = {0};
serial::Serial ros_ser;//定义串口
//创建一个类节点，名字叫做Hand_control,继承自Node.
class Hand_control : public rclcpp::Node 
{
public:
    // 初始化汉堡总数NumOfAll为100
    Hand_control() : Node("Hand_control")
    {
        RCLCPP_INFO(this->get_logger(), "实例化成功");
        // 实例化回调组, 作用为避免死锁(请自行百度ROS2死锁)
        //callback_group_Hand_control = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        //callback_group_getangleact = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // 实例化服务
        Setangle_Server = this->create_service<service_interfaces::srv::Setangle>("Setangle",
                                    std::bind(&Hand_control::setangle_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setangle);
        Setpos_Server = this->create_service<service_interfaces::srv::Setpos>("Setpos",
                                    std::bind(&Hand_control::setpos_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setpos);
        Setspeed_Server = this->create_service<service_interfaces::srv::Setspeed>("Setspeed",
                                    std::bind(&Hand_control::setspeed_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setspeed);
        Setforce_Server = this->create_service<service_interfaces::srv::Setforce>("Setforce",
                                    std::bind(&Hand_control::setforce_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setforce);
                                    
        Setgestureno_Server = this->create_service<service_interfaces::srv::Setgestureno>("Setgestureno",
                                    std::bind(&Hand_control::setgestureno_callback,this,_1,_2));                             
                                    
        Getangleact_Server = this->create_service<service_interfaces::srv::Getangleact>("Getangleact",
                                    std::bind(&Hand_control::getangleact_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getangleact);
        Getangleset_Server = this->create_service<service_interfaces::srv::Getangleset>("Getangleset",
                                    std::bind(&Hand_control::getangleset_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getangleset);
        Getposact_Server = this->create_service<service_interfaces::srv::Getposact>("Getposact",
                                    std::bind(&Hand_control::getposact_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getposact);
        Getposset_Server = this->create_service<service_interfaces::srv::Getposset>("Getposset",
                                    std::bind(&Hand_control::getposset_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getposset);
        Getspeedset_Server = this->create_service<service_interfaces::srv::Getspeedset>("Getspeedset",
                                    std::bind(&Hand_control::getspeedset_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getspeedset);
        Getforceact_Server = this->create_service<service_interfaces::srv::Getforceact>("Getforceact",
                                    std::bind(&Hand_control::getforceact_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getforceact);
        Getforceset_Server = this->create_service<service_interfaces::srv::Getforceset>("Getforceset",
                                    std::bind(&Hand_control::getforceset_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getforceset);
        Getcurrentact_Server = this->create_service<service_interfaces::srv::Getcurrentact>("Getcurrentact",
                                    std::bind(&Hand_control::getcurrentact_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getcurrentact);
        Geterror_Server = this->create_service<service_interfaces::srv::Geterror>("Geterror",
                                    std::bind(&Hand_control::geterror_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_geterror);
        Gettemp_Server = this->create_service<service_interfaces::srv::Gettemp>("Gettemp",
                                    std::bind(&Hand_control::gettemp_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_gettemp);
    }
private:
    // 声明服务回调组
    rclcpp::CallbackGroup::SharedPtr callback_group_setangle;
    rclcpp::CallbackGroup::SharedPtr callback_group_setpos;
    rclcpp::CallbackGroup::SharedPtr callback_group_setspeed;
    rclcpp::CallbackGroup::SharedPtr callback_group_setforce;
    rclcpp::CallbackGroup::SharedPtr callback_group_setgestureno;
    rclcpp::CallbackGroup::SharedPtr callback_group_getangleact;
    rclcpp::CallbackGroup::SharedPtr callback_group_getangleset;
    rclcpp::CallbackGroup::SharedPtr callback_group_getposact;
    rclcpp::CallbackGroup::SharedPtr callback_group_getposset;
    rclcpp::CallbackGroup::SharedPtr callback_group_getspeedset;
    rclcpp::CallbackGroup::SharedPtr callback_group_getforceact;
    rclcpp::CallbackGroup::SharedPtr callback_group_getforceset;
    rclcpp::CallbackGroup::SharedPtr callback_group_getcurrentact;
    rclcpp::CallbackGroup::SharedPtr callback_group_geterror;
    rclcpp::CallbackGroup::SharedPtr callback_group_gettemp;
    // 声明服务端
    rclcpp::Service<service_interfaces::srv::Setangle>::SharedPtr Setangle_Server;
    rclcpp::Service<service_interfaces::srv::Setpos>::SharedPtr Setpos_Server;
    rclcpp::Service<service_interfaces::srv::Setspeed>::SharedPtr Setspeed_Server;
    rclcpp::Service<service_interfaces::srv::Setforce>::SharedPtr Setforce_Server;
    rclcpp::Service<service_interfaces::srv::Setgestureno>::SharedPtr Setgestureno_Server;
    rclcpp::Service<service_interfaces::srv::Getangleact>::SharedPtr Getangleact_Server;
    rclcpp::Service<service_interfaces::srv::Getangleset>::SharedPtr Getangleset_Server;
    rclcpp::Service<service_interfaces::srv::Getposact>::SharedPtr Getposact_Server;
    rclcpp::Service<service_interfaces::srv::Getposset>::SharedPtr Getposset_Server;
    rclcpp::Service<service_interfaces::srv::Getspeedset>::SharedPtr Getspeedset_Server;
    rclcpp::Service<service_interfaces::srv::Getforceact>::SharedPtr Getforceact_Server;
    rclcpp::Service<service_interfaces::srv::Getforceset>::SharedPtr Getforceset_Server;
    rclcpp::Service<service_interfaces::srv::Getcurrentact>::SharedPtr Getcurrentact_Server;
    rclcpp::Service<service_interfaces::srv::Geterror>::SharedPtr Geterror_Server;
    rclcpp::Service<service_interfaces::srv::Gettemp>::SharedPtr Gettemp_Server;
    // 声明回调函数，当收到要请求时调用该函数
    void setangle_callback(const service_interfaces::srv::Setangle::Request::SharedPtr request,
                               const service_interfaces::srv::Setangle::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_angle")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x0F;
            send_buffer[4] = 0x12;
            send_buffer[5] = 0xCE;
            send_buffer[6] = 0x05;
            send_buffer[7] = (request->angle0 & 0xFF);
            send_buffer[8] = ((request->angle0 >> 8) & 0xFF);
            send_buffer[9] = (request->angle1 & 0xFF);
            send_buffer[10] = ((request->angle1 >> 8) & 0xFF);
            send_buffer[11] = (request->angle2 & 0xFF);
            send_buffer[12] = ((request->angle2 >> 8) & 0xFF);
            send_buffer[13] = (request->angle3 & 0xFF);
            send_buffer[14] = ((request->angle3 >> 8) & 0xFF);
            send_buffer[15] = (request->angle4 & 0xFF);
            send_buffer[16] = ((request->angle4 >> 8) & 0xFF);
            send_buffer[17] = (request->angle5 & 0xFF);
            send_buffer[18] = ((request->angle5 >> 8) & 0xFF);
            for(int i = 2;i < 19;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[19] = check_sum;
            ros_ser.write(send_buffer,20);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[7] == 0x01)
                {
                    response->angle_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->angle_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->angle_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    // 声明回调函数，当收到要请求时调用该函数
    void setpos_callback(const service_interfaces::srv::Setpos::Request::SharedPtr request,
                               const service_interfaces::srv::Setpos::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_pos")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x0F;
            send_buffer[4] = 0x12;
            send_buffer[5] = 0xC2;
            send_buffer[6] = 0x05;
            send_buffer[7] = (request->pos0 & 0xFF);
            send_buffer[8] = ((request->pos0 >> 8) & 0xFF);
            send_buffer[9] = (request->pos1 & 0xFF);
            send_buffer[10] = ((request->pos1 >> 8) & 0xFF);
            send_buffer[11] = (request->pos2 & 0xFF);
            send_buffer[12] = ((request->pos2 >> 8) & 0xFF);
            send_buffer[13] = (request->pos3 & 0xFF);
            send_buffer[14] = ((request->pos3 >> 8) & 0xFF);
            send_buffer[15] = (request->pos4 & 0xFF);
            send_buffer[16] = ((request->pos4 >> 8) & 0xFF);
            send_buffer[17] = (request->pos5 & 0xFF);
            send_buffer[18] = ((request->pos5 >> 8) & 0xFF);
            for(int i = 2;i < 19;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[19] = check_sum;
            ros_ser.write(send_buffer,20);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[7] == 0x01)
                {
                    response->pos_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->pos_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->pos_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    // 声明回调函数，当收到要请求时调用该函数
    void setspeed_callback(const service_interfaces::srv::Setspeed::Request::SharedPtr request,
                               const service_interfaces::srv::Setspeed::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_speed")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x0F;
            send_buffer[4] = 0x12;
            send_buffer[5] = 0xF2;
            send_buffer[6] = 0x05;
            send_buffer[7] = (request->speed0 & 0xFF);
            send_buffer[8] = ((request->speed0 >> 8) & 0xFF);
            send_buffer[9] = (request->speed1 & 0xFF);
            send_buffer[10] = ((request->speed1 >> 8) & 0xFF);
            send_buffer[11] = (request->speed2 & 0xFF);
            send_buffer[12] = ((request->speed2 >> 8) & 0xFF);
            send_buffer[13] = (request->speed3 & 0xFF);
            send_buffer[14] = ((request->speed3 >> 8) & 0xFF);
            send_buffer[15] = (request->speed4 & 0xFF);
            send_buffer[16] = ((request->speed4 >> 8) & 0xFF);
            send_buffer[17] = (request->speed5 & 0xFF);
            send_buffer[18] = ((request->speed5 >> 8) & 0xFF);
            for(int i = 2;i < 19;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[19] = check_sum;
            ros_ser.write(send_buffer,20);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[7] == 0x01)
                {
                    response->speed_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->speed_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->speed_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    // 声明回调函数，当收到要请求时调用该函数
    void setforce_callback(const service_interfaces::srv::Setforce::Request::SharedPtr request,
                               const service_interfaces::srv::Setforce::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_force")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x0F;
            send_buffer[4] = 0x12;
            send_buffer[5] = 0xDA;
            send_buffer[6] = 0x05;
            send_buffer[7] = (request->force0 & 0xFF);
            send_buffer[8] = ((request->force0 >> 8) & 0xFF);
            send_buffer[9] = (request->force1 & 0xFF);
            send_buffer[10] = ((request->force1 >> 8) & 0xFF);
            send_buffer[11] = (request->force2 & 0xFF);
            send_buffer[12] = ((request->force2 >> 8) & 0xFF);
            send_buffer[13] = (request->force3 & 0xFF);
            send_buffer[14] = ((request->force3 >> 8) & 0xFF);
            send_buffer[15] = (request->force4 & 0xFF);
            send_buffer[16] = ((request->force4 >> 8) & 0xFF);
            send_buffer[17] = (request->force5 & 0xFF);
            send_buffer[18] = ((request->force5 >> 8) & 0xFF);
            for(int i = 2;i < 19;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[19] = check_sum;
            ros_ser.write(send_buffer,20);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[7] == 0x01)
                {
                    response->force_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->force_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->force_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    
    void setgestureno_callback(const service_interfaces::srv::Setgestureno::Request::SharedPtr request,
                           const service_interfaces::srv::Setgestureno::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0); // 设置循环频率为10Hz

    // 首先判断指令类型
    if (request->status == "set_gesture_no")
    {
        // 打印指令类型
        RCLCPP_INFO(this->get_logger(), "收到%s的请求，手势编号:%d", request->status.c_str(), request->gesture_no);

        // 构建发送缓冲区
        send_buffer[0] = 0xEB; // 起始字节
        send_buffer[1] = 0x90; // 命令字节
        send_buffer[2] = request->hand_id; // 手部ID
        send_buffer[3] = 0x05; // 固定字节
        send_buffer[4] = 0x12; // 固定字节
        send_buffer[5] = 0x10; // 地址低字节
        send_buffer[6] = 0x09; // 地址高字节
        send_buffer[7] = (request->gesture_no & 0xFF); // 手势编号低字节
        send_buffer[8] = ((request->gesture_no >> 8) & 0xFF); // 手势编号高字节

        // 计算校验和
        for (int i = 2; i < 9; i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[9] = check_sum; // 将校验和值放入缓冲区

        // 打印发送缓冲区内容
        RCLCPP_INFO(this->get_logger(), "发送手势编号数据:");
        for (int i = 0; i < 10; i++)
        {
            RCLCPP_INFO(this->get_logger(), "send_buffer[%d]: 0x%02X", i, send_buffer[i]);
        }

        // 发送手势编号到设备
        ros_ser.write(send_buffer, 10);
        loop_rate.sleep(); // 等待100ms接收数据

        int count = ros_ser.available(); // 检查是否有数据可读
        if (count != 0) // 等待接收数据
        {
            std::vector<unsigned char> recv_buffer(count); // 创建接收缓冲区
            count = ros_ser.read(&recv_buffer[0], count); // 读取数据
            if (recv_buffer[7] == 0x01) // 根据接收到的数据判断是否成功
            {
                response->gesture_nograb = true; // 设置成功
                RCLCPP_INFO(this->get_logger(), "设置手势编号成功，手势编号:%d", request->gesture_no);
            }
            else
            {
                response->gesture_nograb = false; // 设置失败
                RCLCPP_ERROR(this->get_logger(), "设置手势编号失败，手势编号:%d", request->gesture_no);
            }
        }
        else
        {
            // 超时或未收到数据
            response->gesture_nograb = false;
            RCLCPP_WARN(this->get_logger(), "未收到响应数据，手势编号:%d", request->gesture_no);
        }

        // 现在设置执行动作序列号到地址2322
        send_buffer[0] = 0xEB; // 重置起始字节
        send_buffer[1] = 0x90; // 重置命令字节
        send_buffer[2] = request->hand_id; // 手部ID
        send_buffer[3] = 0x05; // 固定字节
        send_buffer[4] = 0x12; // 固定字节
        send_buffer[5] = 0x12; // 地址低字节
        send_buffer[6] = 0x09; // 地址高字节
        send_buffer[7] = 0x01; // 执行序列号
        send_buffer[8] = 0x00; // 额外的字节（如果需要）

        // 计算校验和
        check_sum = 0; // 重置校验和
        for (int i = 2; i < 9; i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[9] = check_sum; // 将校验和值放入缓冲区

        // 打印发送执行序列号的缓冲区内容
        RCLCPP_INFO(this->get_logger(), "发送执行动作序列号数据:");
        for (int i = 0; i < 10; i++)
        {
            RCLCPP_INFO(this->get_logger(), "send_buffer[%d]: 0x%02X", i, send_buffer[i]);
        }

        // 发送执行序列号到设备
        ros_ser.write(send_buffer, 10);
        loop_rate.sleep(); // 等待100ms接收数据

        count = ros_ser.available(); // 检查是否有数据可读
        if (count != 0) // 等待接收数据
        {
            std::vector<unsigned char> recv_buffer(count); // 创建接收缓冲区
            count = ros_ser.read(&recv_buffer[0], count); // 读取数据
            if (recv_buffer[7] == 0x01) // 根据接收到的数据判断是否成功
            {
                RCLCPP_INFO(this->get_logger(), "设置执行动作序列号成功");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "设置执行动作序列号失败");
            }
        }
        else
        {
            // 超时或未收到数据
            RCLCPP_WARN(this->get_logger(), "未收到执行动作序列号响应数据");
        }
    }
    else
    {
        // 请求错误
        response->gesture_nograb = false;
        RCLCPP_WARN(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
    }
}


    
    void getangleact_callback(const service_interfaces::srv::Getangleact::Request::SharedPtr request,
                               const service_interfaces::srv::Getangleact::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_angleact")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x0A;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->curangleact[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手实际角度为:%d %d %d %d %d %d", 
                    response->curangleact[0],response->curangleact[1],response->curangleact[2],response->curangleact[3],response->curangleact[4],response->curangleact[5]);
                }
                else
                {
                    printf("无法读取灵巧手实际角度值\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    void getangleset_callback(const service_interfaces::srv::Getangleset::Request::SharedPtr request,
                               const service_interfaces::srv::Getangleset::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_angleset")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xCE;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->curangleset[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手设置角度为:%d %d %d %d %d %d", 
                    response->curangleset[0],response->curangleset[1],response->curangleset[2],response->curangleset[3],response->curangleset[4],response->curangleset[5]);
                }
                else
                {
                    printf("无法读取灵巧手角度设置值\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    void getposact_callback(const service_interfaces::srv::Getposact::Request::SharedPtr request,
                               const service_interfaces::srv::Getposact::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_posact")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xFE;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->curposact[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手驱动器实际位置值为:%d %d %d %d %d %d", 
                    response->curposact[0],response->curposact[1],response->curposact[2],response->curposact[3],response->curposact[4],response->curposact[5]);
                }
                else
                {
                    printf("无法读取灵巧手驱动器实际位置值\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    void getposset_callback(const service_interfaces::srv::Getposset::Request::SharedPtr request,
                               const service_interfaces::srv::Getposset::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_posset")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xC2;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->curposset[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手驱动器设置值为:%d %d %d %d %d %d", 
                    response->curposset[0],response->curposset[1],response->curposset[2],response->curposset[3],response->curposset[4],response->curposset[5]);
                }
                else
                {
                    printf("无法读取灵巧手驱动器设置值\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    void getspeedset_callback(const service_interfaces::srv::Getspeedset::Request::SharedPtr request,
                               const service_interfaces::srv::Getspeedset::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_speedset")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xF2;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->curspeedset[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手设置速度值为:%d %d %d %d %d %d", 
                    response->curspeedset[0],response->curspeedset[1],response->curspeedset[2],response->curspeedset[3],response->curspeedset[4],response->curspeedset[5]);
                }
                else
                {
                    printf("无法读取灵巧手设置速度值\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    void getforceact_callback(const service_interfaces::srv::Getforceact::Request::SharedPtr request,
                               const service_interfaces::srv::Getforceact::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_forceact")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x2E;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->curforceact[i] = int16_t((recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00));
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手实际受力值:%d %d %d %d %d %d", 
                    response->curforceact[0],response->curforceact[1],response->curforceact[2],response->curforceact[3],response->curforceact[4],response->curforceact[5]);
                }
                else
                {
                    printf("无法读取灵巧手实际受力值\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    void getforceset_callback(const service_interfaces::srv::Getforceset::Request::SharedPtr request,
                               const service_interfaces::srv::Getforceset::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_forceset")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0xDA;
            send_buffer[6] = 0x05;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->curforceset[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手设置受力值为:%d %d %d %d %d %d", 
                    response->curforceset[0],response->curforceset[1],response->curforceset[2],response->curforceset[3],response->curforceset[4],response->curforceset[5]);
                }
                else
                {
                    printf("无法读取灵巧手设置受力值\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    void getcurrentact_callback(const service_interfaces::srv::Getcurrentact::Request::SharedPtr request,
                               const service_interfaces::srv::Getcurrentact::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_currentact")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x3A;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x0C;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->curcurrent[i] = (recv_buffer[7+i*2] & 0xFF) + ((recv_buffer[8+i*2]<<8) & 0xFF00);
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手实际电流为:%d %d %d %d %d %d", 
                    response->curcurrent[0],response->curcurrent[1],response->curcurrent[2],response->curcurrent[3],response->curcurrent[4],response->curcurrent[5]);
                }
                else
                {
                    printf("无法读取灵巧手实际电流值\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    void geterror_callback(const service_interfaces::srv::Geterror::Request::SharedPtr request,
                               const service_interfaces::srv::Geterror::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_error")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x46;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x06;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->error[i] = (recv_buffer[7+i] & 0xFF);
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手故障码为:%d %d %d %d %d %d", 
                    response->error[0],response->error[1],response->error[2],response->error[3],response->error[4],response->error[5]);
                }
                else
                {
                    printf("无法读取灵巧故障码\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    void gettemp_callback(const service_interfaces::srv::Gettemp::Request::SharedPtr request,
                               const service_interfaces::srv::Gettemp::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_temp")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->hand_id;
            send_buffer[3] = 0x04;
            send_buffer[4] = 0x11;
            send_buffer[5] = 0x52;
            send_buffer[6] = 0x06;
            send_buffer[7] = 0x06;
            for(int i = 2;i < 8;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[8] = check_sum;
            ros_ser.write(send_buffer,9);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x11)
                {
                    for(int i=0;i<6;i++)
                    { 
                        response->temp[i] = (recv_buffer[7+i] & 0xFF);
                    }
                    RCLCPP_INFO(this->get_logger(), "灵巧手手指温度为:%d %d %d %d %d %d", 
                    response->temp[0],response->temp[1],response->temp[2],response->temp[3],response->temp[4],response->temp[5]);
                }
                else
                {
                    printf("无法读取灵巧手温度值\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //rclcpp::WallRate loop_rate(10.0);
    ros_ser.setPort("/dev/ttyUSB0");
    ros_ser.setBaudrate(115200);
    serial::Timeout to =serial::Timeout::simpleTimeout(1000);
    ros_ser.setTimeout(to);
    try
    {
        ros_ser.open();
    }
    catch(serial::IOException &e)
    {
        std::cout<<"serial unable to open"<<std::endl;
        return -1;
    }
    if(ros_ser.isOpen())
    {
        std::cout<<"serial open success"<<std::endl;
    }
    else
    {
        return -1;
    }
    auto node = std::make_shared<Hand_control>();
    // 把节点的执行器变成多线程执行器, 避免死锁
    rclcpp::executors::MultiThreadedExecutor exector;
    exector.add_node(node);
    exector.spin();
    rclcpp::shutdown();
    return 0;
}

