#include "rclcpp/rclcpp.hpp"

// ================================
// runc class
// ================================
#include "../include/cpp_pkg/Run.hpp"

class CPPNode : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer;

  Run run;
  void timer_callback()
  {
    run.run();
  }

public:
  // ================================
  // constructor should inherit Node
  // ================================
  CPPNode(std::string node_name)
      : Node(node_name)
  {
    RCLCPP_INFO(this->get_logger(), "node creted !");

    timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CPPNode::timer_callback, this));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<CPPNode> node = std::make_shared<CPPNode>("cpp_node");

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
