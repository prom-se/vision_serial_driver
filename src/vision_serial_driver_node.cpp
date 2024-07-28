#include "../include/vision_serial_driver/vision_serial_driver_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <vision_interfaces/msg/auto_aim.hpp>
#include <vision_interfaces/msg/robot.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

vision::SerialDriver::SerialDriver(std::string node_name)
    : rclcpp::Node(node_name), ctx_{IoContext(2)}, serial_driver_{drivers::serial_driver::SerialDriver(ctx_)}
{
    RCLCPP_INFO(get_logger(), "节点:/%s启动", node_name.c_str());
    dev_name_ = declare_parameter("device_name", "/dev/ttyACM0");
    baud_rate_ = declare_parameter("baud_rate", 115200);
    portConfig_ = new drivers::serial_driver::SerialPortConfig(baud_rate_, drivers::serial_driver::FlowControl::NONE, drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);

    using namespace std::chrono_literals;
    // 设置重启计时器1hz.
    reopen_timer_ = create_wall_timer(
        1s, std::bind(&vision::SerialDriver::serial_reopen_callback, this));

    // 设置发布计时器500hz.
    publish_timer_ = create_wall_timer(
        2ms, std::bind(&vision::SerialDriver::robot_callback, this));

    // 发布Robot信息.
    publisher_ = create_publisher<vision_interfaces::msg::Robot>(
        "/serial_driver/robot", rclcpp::SensorDataQoS());

    // 订阅AutoAim信息.
    subscription_ = create_subscription<vision_interfaces::msg::AutoAim>(
        "/serial_driver/aim_target", rclcpp::SensorDataQoS(), std::bind(&vision::SerialDriver::auto_aim_callback, this, std::placeholders::_1));

    // 设置串口读取线程.
    rx_thread_ = std::thread(&vision::SerialDriver::serial_read_thread, this);

    // TF broadcaster
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.006);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Detect parameter client
    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
}

vision::SerialDriver::~SerialDriver()
{
    if (serial_driver_.port()->is_open())
    {
        serial_driver_.port()->close();
    }
    if (rx_thread_.joinable())
    {
        rx_thread_.join();
    }
}

void vision::SerialDriver::serial_reopen_callback()
{
    if (!is_open_)
    {
        dev_name_ = get_parameter("device_name").as_string();
        baud_rate_ = get_parameter("baud_rate").as_int();
        portConfig_ = new drivers::serial_driver::SerialPortConfig(baud_rate_, drivers::serial_driver::FlowControl::NONE, drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);
        try
        {
            RCLCPP_WARN(get_logger(), "重启串口:%s...", dev_name_.c_str());
            serial_driver_.init_port(dev_name_, *portConfig_);
            serial_driver_.port()->open();
            is_open_ = serial_driver_.port()->is_open();
        }
        catch (const std::system_error &error)
        {
            RCLCPP_ERROR(get_logger(), "打开串口:%s失败", dev_name_.c_str());
            is_open_ = false;
        }
        if (is_open_)
            RCLCPP_INFO(get_logger(), "打开串口:%s成功", dev_name_.c_str());
    }
}

void vision::SerialDriver::serial_read_thread()
{
    while (rclcpp::ok())
    {
        std::vector<uint8_t> robot_data(sizeof(robot_pack_.bytes));
        if (is_open_)
        {
            try
            {
                serial_driver_.port()->receive(robot_data);
                if (robot_data[0] == 0xA5 && robot_data[1] == 0x00)
                { // 包头为0xA5
                    memcpy(robot_pack_.bytes, robot_data.data(), sizeof(robot_pack_.bytes));
                }
            }
            catch (const std::exception &error)
            {
                RCLCPP_ERROR(get_logger(), "读取串口时发生错误.");
                is_open_ = false;
            }
        }
    }
}

void vision::SerialDriver::serial_write(uint8_t *data, size_t len)
{
    try
    {
        serial_driver_.port()->send(std::vector<uint8_t>(data, data + len));
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(get_logger(), "写入串口时发生错误.");
        is_open_ = false;
    }
}

void vision::SerialDriver::auto_aim_callback(const vision_interfaces::msg::AutoAim vMsg)
{
    if (is_open_)
    {
        vision_pack_.msg.head = 0x00A5;
        vision_pack_.msg.fire = vMsg.fire;
        vision_pack_.msg.aim_pitch = vMsg.aim_pitch;
        vision_pack_.msg.aim_yaw = vMsg.aim_yaw;
        serial_write(vision_pack_.bytes, sizeof(vision_pack_.bytes));
    }
}

void vision::SerialDriver::robot_callback()
{
    if (is_open_)
    {
        try
        {
            auto msg = vision_interfaces::msg::Robot();
            msg.foe_color = robot_pack_.msg.foe_color == 1 ? 1 : 0;
            msg.foe_color = robot_pack_.msg.foe_color;
            msg.self_yaw = robot_pack_.msg.robot_yaw;
            msg.self_pitch = robot_pack_.msg.robot_pitch;
            double muzzle_speed = 15.0;
            msg.muzzle_speed = muzzle_speed;
            publisher_->publish(msg);

            geometry_msgs::msg::TransformStamped t;
            timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
            t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
            t.header.frame_id = "odom";
            t.child_frame_id = "gimbal_link";
            tf2::Quaternion q;
            q.setRPY(0, -robot_pack_.msg.robot_pitch * 3.1415926535 / 180.0, robot_pack_.msg.robot_yaw * 3.1415926535 / 180.0);
            t.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(t);

            if (!initial_set_param_ || msg.foe_color != previous_receive_color_)
            {
                auto param = rclcpp::Parameter("detect_color", msg.foe_color);
                if (!detector_param_client_->service_is_ready())
                {
                    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
                    return;
                }
                if (
                    !set_param_future_.valid() ||
                    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
                {
                    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
                    set_param_future_ = detector_param_client_->set_parameters(
                        {param}, [this, param](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> &results)
                        {
              for (const auto & result : results.get()) {
                if (!result.successful) {
                  RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                  return;
                }
              }
              RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
              initial_set_param_ = true; });
                }
                previous_receive_color_ = msg.foe_color;
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 20, "处理串口数据时发生错误: %s", ex.what());
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vision::SerialDriver>("serial");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}