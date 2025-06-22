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
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        param_desc.description = "A simple parameter for demonstration purposes.";
        param_desc.additional_constraints = "Must be a string.";

        this->declare_parameter("my_parameter",  my_param_, param_desc);

        auto timer_callback = [this]()
        {
            std::string my_param = this->get_parameter("my_parameter").as_string();

            RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

            // // Set the parameter to a new value every second
            // std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "earth")};
            // this->set_parameters(all_new_parameters);
        };

        param_callback_handle_ = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
            auto result = rcl_interfaces::msg::SetParametersResult();
            result.successful = true;

            for (const auto &param : params) {
                if (param.get_name() == "my_parameter") {
                    if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                        result.successful = false;
                        result.reason = "my_parameter must be a string";
                        break;
                    }

                    try {
                        RCLCPP_INFO(this->get_logger(), "Updating my_parameter to: %s",
                        param.as_string().c_str());
                        this->my_param_ = param.as_string();
                    } catch (const std::exception &e) {
                        result.successful = false;
                        result.reason = std::string("Failed to set parameter: ") + e.what();
                    }
                }
            }
            return result;
        });

        timer_ = this->create_wall_timer(1000ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::string my_param_ = "world";  // Default value for the parameter
    // Thêm dòng này để lưu callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalParam>());
    rclcpp::shutdown();
    return 0;
}