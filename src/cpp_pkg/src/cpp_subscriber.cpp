#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"

using Int32 = std_msgs::msg::Int32;
using Empty = std_srvs::srv::Empty;

class SubscriberNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Service<Empty>::SharedPtr service_;

    void subscriber(const Int32 &msg)
    {
        RCLCPP_INFO(this->get_logger(), "message %d reveived", msg.data);
        publisher(msg.data);
    }

    void publisher(int num)
    {
        Int32 message = Int32();
        message.data = num;
        publisher_->publish(message);
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

        publisher_ = this->create_publisher<Int32>(
            "second_topic",
            10);
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
