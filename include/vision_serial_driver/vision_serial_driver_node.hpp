#ifndef VISION_SERIAL_DRIVER_HPP
#define VISION_SERIAL_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <vision_interfaces/msg/auto_aim.hpp>
#include <vision_interfaces/msg/robot.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "packet.h"

namespace vision
{
    class SerialDriver : public rclcpp::Node
    {
    public:
        /*
        @brief 串口驱动节点构造函数
        @param[in] node_name 节点名称
        */
        SerialDriver(std::string node_name);

        /*@brief 串口驱动节点析构函数*/
        ~SerialDriver();

    private:
        /*@brief 串口重启回调函数*/
        void serial_reopen_callback();

        /*@brief 串口读取线程函数*/
        void serial_read_thread();

        /*@brief 串口写入函数*/
        void serial_write(uint8_t *data, size_t len);

        /*
        @brief 自瞄回调函数
        @param vMsg 自瞄信息
        */
        void auto_aim_callback(const vision_interfaces::msg::AutoAim vMsg);

        /*@brief 机器人状态回调函数*/
        void robot_callback();

        std::string dev_name_;
        int32_t baud_rate_;
        std::thread rx_thread_;
        rclcpp::TimerBase::SharedPtr reopen_timer_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::Publisher<vision_interfaces::msg::Robot>::SharedPtr publisher_;
        rclcpp::Subscription<vision_interfaces::msg::AutoAim>::SharedPtr subscription_;
        vision::VisionPack vision_pack_;
        vision::RobotPack robot_pack_;

        bool is_open_ = false;
        drivers::serial_driver::SerialPortConfig *portConfig_;
        IoContext ctx_;
        drivers::serial_driver::SerialDriver serial_driver_;

        bool initial_set_param_ = false;
        uint8_t previous_receive_color_ = 0;
        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> set_param_future_;

        double timestamp_offset_ = 0;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    };
}

#endif // VISION_SERIAL_DRIVER_HPP