#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

class CppService : public rclcpp::Node
{
private:
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    void service_callback(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        const std_srvs::srv::SetBool::Response::SharedPtr response)
    {
        response->success = request->data;
        response->message = "flag seted successfully !";
        RCLCPP_INFO(this->get_logger(), "flag seted to %d", response->success);
    }

public:
    // ================================
    // constructor should inherit Node
    // ================================
    CppService()
        : Node("cpp_service_node")
    {
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "first_service",
            std::bind(
                &CppService::service_callback,
                this, std::placeholders::_1,
                std::placeholders::_2));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<CppService> node = std::make_shared<CppService>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
