#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class PublisherNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_{0};
    int frequency_{0};
    std::vector<int64_t> test{0, 0, 0};

    // ================================
    // publisher
    // ================================
    void publisher()
    {
        std_msgs::msg::Int32 msg = std_msgs::msg::Int32();
        msg.data = counter_;
        counter_++;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "publish -> %d", msg.data);
    }

public:
    // ================================
    // constructor should inherit Node
    // ================================
    PublisherNode()
        : Node("cpp_publisher_node"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "node creted !");

        this->declare_parameter("frequency", 1000);
        frequency_ = this->get_parameter("frequency").as_int();

        this->declare_parameter("test", std::vector<int64_t>{0, 0, 1});
        test = this->get_parameter("test").as_integer_array();
        for (int i = 0; i < 3; i++)
        {
            RCLCPP_INFO(this->get_logger(), "i index -> %ld", test[i]);
        }

        publisher_ = this->create_publisher<std_msgs::msg::Int32>(
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
