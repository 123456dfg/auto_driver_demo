// std
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <rclcpp/executors.hpp>
#include <thread>
// ros2
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace auto_serial_driver
{
    class virtualSerialDriver : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        void twistSubCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "receive linear_x: %f linear_y: %f linear_z: %f", msg->linear.x, msg->linear.y, msg->linear.z);
            RCLCPP_INFO(this->get_logger(), "receive angular: %f angular: %f angular: %f", msg->angular.x, msg->angular.y, msg->angular.z);
        }

    public:
        explicit virtualSerialDriver(const rclcpp::NodeOptions options) : Node("virtual_serial_driver", options)
        {
            RCLCPP_INFO(this->get_logger(), "Starting virtual_serial_driver!");
            twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS(), std::bind(&virtualSerialDriver::twistSubCallback, this, std::placeholders::_1));
        };
    };

} // namespace auto_serial_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_serial_driver::virtualSerialDriver)
