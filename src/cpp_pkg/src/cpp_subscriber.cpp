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

    bool active{true};

    void subscriber(const Int32 &msg)
    {
        RCLCPP_INFO(this->get_logger(), "message %d reveived", msg.data);
        publisher(active ? msg.data : 0);
    }

    void publisher(int num)
    {
        Int32 message = Int32();
        message.data = num;
        RCLCPP_INFO(this->get_logger(), "publishing in second topic %d", num);
        publisher_->publish(message);
    }

    void service_callback(
        const std_srvs::srv::Empty::Request::SharedPtr request,
        const std_srvs::srv::Empty::Response::SharedPtr response)
    {
        active = !active;
    }

public:
    // ================================
    // constructor should inherit Node
    // ================================
    SubscriberNode()
        : Node("cpp_subscriber_node")
    {
        subscriber_ = this->create_subscription<Int32>(
            "first_topic",
            10,
            std::bind(&SubscriberNode::subscriber, this, std::placeholders::_1));

        publisher_ = this->create_publisher<Int32>(
            "second_topic",
            10);

        service_ = this->create_service<Empty>(
            "first_service",
            std::bind(
                &SubscriberNode::service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
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
