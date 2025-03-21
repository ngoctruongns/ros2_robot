#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using tutorial_interfaces::msg::Num;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        auto topic_callback =
            [this](tutorial_interfaces::msg::Num::UniquePtr msg) -> void
        {
            RCLCPP_INFO(this->get_logger(), "I heard: %ld", msg->num);
        };
        subscription_ =
            this->create_subscription<Num>("topic", 10, topic_callback);
    }

private:
    rclcpp::Subscription<Num>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}