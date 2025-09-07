#include "rclcpp/rclcpp.hpp"
// #include "example_interfaces/srv/add_two_ints.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"

using AddThreeInts = tutorial_interfaces::srv::AddThreeInts;

#include <memory>

void add(const std::shared_ptr<AddThreeInts::Request> request,
    std::shared_ptr<AddThreeInts::Response>      response)
{
response->sum = request->a + request->b + request->c;
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Input\na: %ld, b: %ld, c: %ld",
            request->a, request->b, request->c);
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Output: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");
    auto service = node->create_service<AddThreeInts>("add_three_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}