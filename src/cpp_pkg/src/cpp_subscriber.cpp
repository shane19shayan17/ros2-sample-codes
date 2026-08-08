#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"

class SubscriberNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr publisher_;

    int num;

    void publisher_callback()
    {
        auto msg = example_interfaces::msg::Int32();
        msg.data = num;

        RCLCPP_INFO(this->get_logger(), "input msg -> %d", msg.data);
        publisher_->publish(msg);
    }

    void subscriber(const example_interfaces::msg::Int32 &msg)
    {
        num = msg.data;
        publisher_callback();
    }

public:
    // ================================
    // constructor should inherit Node
    // ================================
    SubscriberNode(std::string node_name)
        : Node(node_name), num(0)
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int32>(
            "python_publisher",
            5,
            std::bind(&SubscriberNode::subscriber, this, std::placeholders::_1));

        publisher_ = this->create_publisher<example_interfaces::msg::Int32>(
            "second_topic", 5);

        // timer_ = this->create_wall_timer(
        // std::chrono::milliseconds(1000),
        // std::bind(&SubscriberNode::publisher_callback, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<SubscriberNode> node = std::make_shared<SubscriberNode>("cpp_subscriber");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
