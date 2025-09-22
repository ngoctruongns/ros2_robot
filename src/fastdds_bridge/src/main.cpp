#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// FastDDS includes
#include "HelloWorldPubSubTypes.h"
#include "HelloWorldSubscriber.h" // Your custom FastDDS subscriber implementation

using namespace std::chrono_literals;

// ROS2 Node that bridges FastDDS data to ROS2 topic
class HelloBridgeNode : public rclcpp::Node
{
public:
    HelloBridgeNode()
        : Node("hello_bridge_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("fastdds_helloworld", 10);

        // Start FastDDS subscriber in a separate thread
        // std::thread t1(func, arg1, arg2 ...  ); // Example of starting a thread
        fastdds_thread_ = std::thread([this]() { this->run_fastdds_subscriber(); });
    }

    ~HelloBridgeNode()
    {
        if (fastdds_thread_.joinable())
            fastdds_thread_.join();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::thread fastdds_thread_;

    void run_fastdds_subscriber()
    {
        HelloWorldSubscriber subscriber;
        subscriber.init();

        // Lambda to handle received FastDDS messages
        subscriber.set_callback([this](const HelloWorld& msg) {
            auto ros_msg = std_msgs::msg::String();
            ros_msg.data = msg.message(); // Assuming HelloWorld has a 'message' field
            RCLCPP_INFO(this->get_logger(), "Bridged FastDDS: %s", ros_msg.data.c_str());
            publisher_->publish(ros_msg);
        });

        subscriber.run(); // This should block and handle FastDDS events
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
