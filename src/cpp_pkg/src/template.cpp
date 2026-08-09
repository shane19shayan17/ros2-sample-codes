#include "rclcpp/rclcpp.hpp"

#include "../include/cpp_pkg/Logger.hpp"

class TemplateNode : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer;

  void timer_callback()
  {
  }

public:
  // ================================
  // constructor should inherit Node
  // ================================
  TemplateNode()
      : Node("template")
  {
    RCLCPP_INFO(this->get_logger(), "node creted !");

    timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&TemplateNode::timer_callback, this));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<TemplateNode> node = std::make_shared<TemplateNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
