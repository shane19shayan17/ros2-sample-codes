#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SubscriberNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;

    void subscriber(const std_msgs::msg::Int32 &msg)
    {
        RCLCPP_INFO(this->get_logger(), "message %d reveived", msg.data);
    }

public:
    // ================================
    // constructor should inherit Node
    // ================================
    SubscriberNode()
        : Node("cpp_subscriber_node")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
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
