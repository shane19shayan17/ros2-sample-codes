#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"

class PublisherNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_{0};

    // ================================
    // publisher
    // ================================
    void publisher()
    {
        auto msg = example_interfaces::msg::Int32();
        msg.data = counter_;
        counter_++;
        
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "input msg -> %d", msg.data);
    }

public:
    // ================================
    // constructor should inherit Node
    // ================================
    PublisherNode(std::string node_name)
        : Node(node_name), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "node creted !");

        publisher_ = this->create_publisher<example_interfaces::msg::Int32>(
            "python_publisher",
            5);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&PublisherNode::publisher, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<PublisherNode> node = std::make_shared<PublisherNode>("cpp_publisher");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
