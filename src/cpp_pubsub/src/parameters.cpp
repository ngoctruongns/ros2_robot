#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
    public:
        MinimalParam()
        : Node("minimal_param_node")
        {
            this->declare_parameter("my_param", "world");
            // this->get_parameter("my_param", my_param_);
            // RCLCPP_INFO(this->get_logger(), "Parameter value: %s", my_param_.c_str());
            auto timer_callback = [this]() -> void
            {
                std::string my_param = this->get_parameter("my_param").as_string();
                RCLCPP_INFO(this->get_logger(), "Parameter value: %s", my_param.c_str());
                std::vector<rclcpp::Parameter> parameters{
                    rclcpp::Parameter("my_param", "world"),
                };
                this->set_parameters(parameters);
            };
            timer_ =this->create_wall_timer(1000ms, timer_callback);
        };

    private:
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalParam>());
    rclcpp::shutdown();

    return 0;
}