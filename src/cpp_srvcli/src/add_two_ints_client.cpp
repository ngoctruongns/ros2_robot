#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using example_interfaces::srv::AddTwoInts;

// Using code as class node
class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient()
        : Node("add_two_ints_client")
    {
        client_ = this->create_client<AddTwoInts>("add_two_ints");
    }

    bool waitingService(void)
    {
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        return true;
    }

    void sendRequest(int64_t a, int64_t b)
    {
        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        auto result_future = client_->async_send_request(request);

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Sum: %ld", result_future.get()->sum);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
        }
    }

private:
    rclcpp::Client<AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return 1;
    }

    std::unique_ptr<AddTwoIntsClient> node = std::make_unique<AddTwoIntsClient>();

    int64_t a, b;
    try
    {
        a = std::stoll(argv[1]);
        b = std::stoll(argv[2]);
    }
    catch (const std::invalid_argument &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid arguments: %s", e.what());
        return 1;
    }
    catch (const std::out_of_range &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arguments out of range: %s", e.what());
        return 1;
    }

    if (!node->waitingService())
    {
        rclcpp::shutdown();
        return 1;
    }

    node->sendRequest(a, b);

    rclcpp::shutdown();
    return 0;
}