#ifndef VISION_SERIAL_DRIVER_HPP
#define VISION_SERIAL_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <vision_interfaces/msg/auto_aim.hpp>
#include <vision_interfaces/msg/robot.hpp>
#include "packet.h"

using namespace drivers::serial_driver;
using namespace std::chrono_literals;

class serial_driver_node : public rclcpp::Node{
public:
    /*
    @brief 串口驱动节点构造函数
    @param[in] device_name 串口名称
    @param[in] node_name 节点名称
    */
    serial_driver_node(std::string device_name,std::string node_name);

    /*@brief 串口驱动节点析构函数*/
    ~serial_driver_node();
private:
    /*@brief 串口重启回调函数*/
    void serial_reopen_callback();

    /*@brief 串口读取线程函数*/
    void serial_read_thread();

    /*@brief 串口写入函数*/
    void serial_write();

    /*
    @brief 自瞄回调函数
    @param vMsg 自瞄信息
    */
    void auto_aim_callback(const vision_interfaces::msg::AutoAim vMsg);

    /*@brief 机器人状态回调函数*/
    void robot_callback();

    visionArray* vArray;
    robotArray*  rArray;
    bool isOpen=false;
    const std::string dev_name;
    std::thread serialReadThread;
    SerialPortConfig* portConfig;
    IoContext ctx;
    SerialDriver serialDriver=SerialDriver(ctx);

    rclcpp::TimerBase::SharedPtr reopenTimer;
    rclcpp::TimerBase::SharedPtr publishTimer;
    rclcpp::Publisher<vision_interfaces::msg::Robot>::SharedPtr publisher;
    rclcpp::Subscription<vision_interfaces::msg::AutoAim>::SharedPtr subscription;
};

#endif//VISION_SERIAL_DRIVER_HPP