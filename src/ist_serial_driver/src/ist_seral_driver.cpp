#include <rclcpp/logger.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include "serial_driver/serial_driver.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ist_serial_driver/ist_serial_driver.hpp"
#include "ist_serial_driver/cmd.hpp"

namespace ist_serial_driver
{
ISTSerialDriver::ISTSerialDriver(const rclcpp::NodeOptions &options)
: Node("ist_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
    RCLCPP_INFO(get_logger(), "Start ISTSerialDriver!");

    getParams();

    // Create Publisher
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

    enemy_color_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/enemy_color", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pre_point", 10);

    other_latency = this->declare_parameter("other_latency", 0.0);
    // other_latency_ = get_parameter("other_latency").as_double();

    try {
        serial_driver_->init_port(device_name_, *device_config_);
        std::cout << serial_driver_->port()->is_open() << std::endl;
        if (!serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
            std::cout << serial_driver_->port()->is_open() << std::endl;
            receive_thread_ = std::thread(&ISTSerialDriver::receiveData, this);
        }
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
        throw ex;
    }

    pre_point_.header.frame_id = "odom";
    pre_point_.ns = "pre_point";
    pre_point_.type = visualization_msgs::msg::Marker::SPHERE;
    pre_point_.action = visualization_msgs::msg::Marker::ADD;
    pre_point_.scale.x = pre_point_.scale.y = pre_point_.scale.z = 0.12;
    pre_point_.color.a = 1.0;
    pre_point_.color.r = 1.0;
    pre_point_.color.g = 1.0;
    pre_point_.color.b = 1.0;
    pre_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // Create Subscription
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/processor/target", rclcpp::SensorDataQoS(),
        std::bind(&ISTSerialDriver::sendData, this, std::placeholders::_1));
}

ISTSerialDriver::~ISTSerialDriver()
{
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
    }

    if (owned_ctx_) {
        owned_ctx_->waitForExit();
    }
}

void ISTSerialDriver::receiveData()
{
    std::vector<uint8_t> data;
    data.resize(CMD_TO_CV_SIZE);

    while (rclcpp::ok()) {
        try {
                serial_driver_->port()->receive(data);

                int error = CmdToCv_parse(&receivepacket, data.data(), CMD_TO_CV_SIZE);
                if (error == 0) {
                    sensor_msgs::msg::JointState joint_state;
                    joint_state.header.stamp = this->now();
                    joint_state.name.push_back("pitch_joint");
                    joint_state.name.push_back("yaw_joint");
                    joint_state.position.push_back(receivepacket.pitch * M_PI / 180.0);
                    joint_state.position.push_back(receivepacket.yaw * M_PI / 180.0);
                    joint_state_pub_->publish(joint_state);
                    
                    // std::cout << "rec.bullet_speed:" << receivepacket.bullet_speed << std::endl;

                    std_msgs::msg::UInt8 enemy_color;
                    enemy_color.data = receivepacket.enemy_color;
                    enemy_color_pub_->publish(enemy_color);
                } else {
                    RCLCPP_ERROR(get_logger(), "Data parse failed!");
                }
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error while receiving data: %s", ex.what());
            reopenPort();
        }
    }
}

void ISTSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
    // other_latency_ = get_parameter("other_latency").as_double();

    // TODO: 加弹道解算，预测点解算，xyz转py
    double distance = sqrt(pow(msg->position.x, 2) + pow(msg->position.y, 2) + pow(msg->position.z, 2));
    double time  = distance / receivepacket.bullet_speed + other_latency + other_latency;
    
    double x_pre = msg->position.x + msg->velocity.x * time;
    double y_pre = msg->position.y + msg->velocity.y * time;
    double z_pre = msg->position.z + msg->velocity.z * time;

    // std::cout << msg->position.x << std::endl;
    // std::cout << msg->velocity.x << std::endl;

    if (abs(x_pre) > 0.01) {
        pre_point_.header.stamp = this->now();
        pre_point_.pose.position.x = x_pre;
        pre_point_.pose.position.y = y_pre;
        pre_point_.pose.position.z = z_pre;
        marker_pub_->publish(pre_point_);
    }

    float pitch = atan2(z_pre, sqrt(pow(x_pre, 2) + pow(y_pre, 2))) * 180.0 / M_PI - receivepacket.pitch;
    float yaw = atan2(y_pre, x_pre) * 180.0 / M_PI - receivepacket.yaw;

    // std::cout << "x:" << x_pre << std::endl;
    // std::cout << "y:" << y_pre << std::endl;
    // std::cout << "z:" << z_pre << std::endl;
    // std::cout << "rec.pitch:" << receivepacket.pitch << std::endl;
    // std::cout << "rec.yaw:" << receivepacket.yaw << std::endl;
    // std::cout << "cal.pitch:" << atan2(z_pre, sqrt(pow(x_pre, 2) + pow(y_pre, 2))) * 180.0 / M_PI << std::endl;
    // std::cout << "cal.yaw:" << atan2(y_pre, x_pre) * 180.0 / M_PI << std::endl;

    try {
        // 抬头增，顺时针减
        if (msg->position.x == 0 && msg->position.y == 0 && msg->position.z == 0) {
            sendpacket.fire = msg->suggest_fire;
            sendpacket.pitch = 0;
            sendpacket.yaw = 0;
        } else {
            sendpacket.fire = msg->suggest_fire;
            sendpacket.pitch = pitch;   // 英雄加负号
            sendpacket.yaw = yaw;
        }

        // std::cout << "pitch:" << sendpacket.pitch << std::endl;
        // std::cout << "yaw:" << sendpacket.yaw << std::endl;

        std::vector<uint8_t> data;
        data.resize(CMD_TO_EC_SIZE);
        CmdToEc_make(&sendpacket, data.data());

        serial_driver_->port()->send(data);

        std_msgs::msg::Float64 latency;
        latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
        // std::cout << "latency=" << latency.data << std::endl;
        latency_pub_->publish(latency);
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
        reopenPort();
    }
}

void ISTSerialDriver::getParams()
{
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try {
        device_name_ = declare_parameter<std::string>("device_name", "");
    } catch (rclcpp::ParameterTypeException &ex) {
        RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
        throw ex;
    }

    try {
        baud_rate = declare_parameter<int>("baud_rate", 0);
    } catch (rclcpp::ParameterTypeException &ex) {
        RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
        throw ex;
    }

    try {
        const auto fc_string = declare_parameter<std::string>("flow_control", "none");

        if (fc_string == "none") {
            fc = FlowControl::NONE;
        } else if (fc_string == "hardware") {
            fc = FlowControl::HARDWARE;
        } else if (fc_string == "software") {
            fc = FlowControl::SOFTWARE;
        } else {
            throw std::invalid_argument{
            "The flow_control parameter must be one of: none, software, or hardware."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
        throw ex;
    }

    try {
        const auto pt_string = declare_parameter<std::string>("parity", "none");

        if (pt_string == "none") {
            pt = Parity::NONE;
        } else if (pt_string == "odd") {
            pt = Parity::ODD;
        } else if (pt_string == "even") {
            pt = Parity::EVEN;
        } else {
            throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
        throw ex;
    }

    try {
        const auto sb_string = declare_parameter<std::string>("stop_bits", "");

        if (sb_string == "1" || sb_string == "1.0") {
            sb = StopBits::ONE;
        } else if (sb_string == "1.5") {
            sb = StopBits::ONE_POINT_FIVE;
        } else if (sb_string == "2" || sb_string == "2.0") {
            sb = StopBits::TWO;
        } else {
            throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
        throw ex;
    }

    device_config_ =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void ISTSerialDriver::reopenPort()
{
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try {
        if (serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(get_logger(), "Successfully reopened port");
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
        if (rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            reopenPort();
        }
    }
}

}   // namespace ist_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ist_serial_driver::ISTSerialDriver)