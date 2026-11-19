#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class ThreeJointStatePublisher : public rclcpp::Node
{
public:
    ThreeJointStatePublisher() : Node("three_joint_state_publisher")
    {
        // Publisher for the standard /joint_states topic
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Publish every 20 ms (50 Hz)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ThreeJointStatePublisher::timer_callback, this));

        start_time_ = this->get_clock()->now();
    }

private:
    void timer_callback()
    {
        auto joint_state = sensor_msgs::msg::JointState();

        // Update timestamp
        joint_state.header.stamp = this->get_clock()->now();

        joint_state.name = {"base_to_link_1", "link_1_link_2", "link_2_link_3"};

        // Calculate elapsed time for smooth animation (radians)
        auto elapsed = (this->get_clock()->now() - start_time_).seconds();

        // joint_state.position = {
        //     std::sin(elapsed),       // Joint 1
        //     std::cos(elapsed) * 0.25, // Joint 2
        //     std::sin(elapsed * 0.25)  // Joint 3
        // };

        joint_state.position = {
            2.49995991e-01,  // Joint 1
            -1.29216447e-05, // Joint 2
            3.00024239e-01   // Joint 3
        };

        // Publish the message
        publisher_->publish(joint_state);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThreeJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}