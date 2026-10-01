#include "rclcpp/rclcpp.hpp"
#include "messages/msg/velocity.hpp"

using Velocity = messages::msg::Velocity;

class PublisherNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<Velocity>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_{0};
    int frequency_{0};

    // ================================
    // publisher
    // ================================
    void publisher()
    {
        Velocity msg = Velocity();
        msg.velocity_x = counter_ * 100.0;
        msg.velocity_y = counter_ * 10.0;
        msg.velocity_z = counter_ * 1.0;
        counter_++;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "publish -> x: %f, y: %f, z: %f",
                    msg.velocity_x,
                    msg.velocity_y,
                    msg.velocity_z);
    }

public:
    // ================================
    // constructor should inherit Node
    // ================================
    PublisherNode()
        : Node("cpp_publisher_node"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "node creted !");

        this->declare_parameter<int>("frequency", 1000);
        frequency_ = this->get_parameter("frequency").as_int();

        publisher_ = this->create_publisher<Velocity>(
            "first_topic",
            10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(frequency_),
            std::bind(&PublisherNode::publisher, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<PublisherNode> node = std::make_shared<PublisherNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
