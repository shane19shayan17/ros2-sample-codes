#include "../../include/cpp_pkg/Run.hpp"

Run::Run()
{
}

void Run::run()
{
    RCLCPP_INFO(rclcpp::get_logger("cpp_node").get_child("Run"), "hello from run");
}