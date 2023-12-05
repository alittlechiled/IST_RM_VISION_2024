#ifndef IST_SERIAL_DRIVER__IST_SERIAL_DRIVER_HPP_
#define IST_SERIAL_DRIVER__IST_SERIAL_DRIVER_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "serial_driver/serial_driver.hpp"
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"
#include "ist_serial_driver/cmd.hpp"

#include <std_msgs/msg/u_int8.hpp>

namespace ist_serial_driver
{
class ISTSerialDriver : public rclcpp::Node
{
public:
    explicit ISTSerialDriver(const rclcpp::NodeOptions &options);

    ~ISTSerialDriver() override;

private:
    void getParams();

    void receiveData();

    void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

    void reopenPort();

    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr enemy_color_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

    visualization_msgs::msg::Marker pre_point_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::thread receive_thread_;

    CmdToCv receivepacket;
    CmdToEc sendpacket;

    double other_latency;
    // double other_latency_;
};

}   // namespace ist_serial_driver

#endif  // IST_SERIAL_DRIVER__IST_SERIAL_DRIVER_HPP_