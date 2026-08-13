#include "rclcpp/rclcpp.hpp"
#include "messages/msg/velocity.hpp"

using Velocity = messages::msg::Velocity;

class SubscriberNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<Velocity>::SharedPtr subscriber_;

    bool active{true};

    void subscriber(const Velocity &msg)
    {
        RCLCPP_INFO(this->get_logger(), "message x:%f, y:%f, z:%f reveived", msg.velocity_x, msg.velocity_y, msg.velocity_z);
    }

public:
    // ================================
    // constructor should inherit Node
    // ================================
    SubscriberNode()
        : Node("cpp_subscriber_node")
    {
        subscriber_ = this->create_subscription<Velocity>(
            "first_topic",
            10,
            std::bind(&SubscriberNode::subscriber, this, std::placeholders::_1));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SubscriberNode> node = std::make_shared<SubscriberNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
