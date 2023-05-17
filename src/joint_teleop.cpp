// Description: A node that subscribes to joint states and publishes joint position command
#include <array>
#include <mutex>
#include <cmath>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include "franka_streaming_driver/msg/franka_joint_state.hpp"
#include "franka_streaming_driver/msg/franka_joint_cmd.hpp"

using namespace std::chrono_literals;

class JointTeleop : public rclcpp::Node
{
public:
    JointTeleop()
        : Node("joint_teleop")
    {
        subscriber_ = this->create_subscription<franka_streaming_driver::msg::FrankaJointState>(
            "/franka_joint_states", 1, std::bind(&JointTeleop::state_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<franka_streaming_driver::msg::FrankaJointCmd>("/franka_joint_commands", 1);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&JointTeleop::publish_command, this));
        message_.name.resize(7);
        message_.position.resize(7);
        message_.velocity.resize(7);
        message_.effort.resize(7);
        message_.kp.resize(7);
        message_.kd.resize(7);
    }
    ~JointTeleop()
    {
        is_state_initialized_ = false;
        RCLCPP_INFO(this->get_logger(), "Shutting down");
        subscriber_.reset();
        publisher_.reset();
    }

private:
    // TODO: Make this a realtime safe publisher using realtime tool
    void publish_command()
    {
        if (!is_state_initialized_)
        {
            return;
        }

        auto time = this->get_clock()->now() - start_time_;
        auto delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time.seconds()));
        q_d_ = init_q_;
        q_d_.at(4) += delta_angle;

        message_.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

        for (size_t i = 0; i < 7; i++)
        {
            message_.name[i] = joint_names_[i];
            message_.position[i] = q_d_[i];
            message_.velocity[i] = 0.0;
            message_.effort[i] = 0.0;
            message_.kp[i] = Kp_[i];
            message_.kd[i] = Kd_[i];
        }

        publisher_->publish(message_);
    }

    // TODO: Make this lock free using realtime tools
    void state_callback(const franka_streaming_driver::msg::FrankaJointState::SharedPtr msg)
    {
        std::copy(msg->position.begin(), msg->position.end(), q_.begin());
        std::copy(msg->velocity.begin(), msg->velocity.end(), dq_.begin());
        std::copy(msg->tau.begin(), msg->tau.end(), tau_J_.begin());
        std::copy(msg->tau_ext.begin(), msg->tau_ext.end(), tau_J_ext_.begin());
        if (!is_state_initialized_)
        {
            init_q_ = q_;
            q_d_ = q_;
            start_time_ = this->get_clock()->now();
            is_state_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Initialized state");
        }
    }

    rclcpp::Publisher<franka_streaming_driver::msg::FrankaJointCmd>::SharedPtr publisher_;
    rclcpp::Subscription<franka_streaming_driver::msg::FrankaJointState>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_state_initialized_ = false;

    // variables storing the robot state
    std::array<double, 7> q_d_;
    std::array<double, 7> init_q_;
    std::array<double, 7> q_;
    std::array<double, 7> dq_;
    std::array<double, 7> tau_J_;
    std::array<double, 7> tau_J_ext_;
    std::array<double, 7> Kp_ = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    std::array<double, 7> Kd_ = {{50.0, 50.0, 50.0, 50.0, 20.0, 20.0, 15.0}};
    rclcpp::Time start_time_;
    franka_streaming_driver::msg::FrankaJointCmd message_;

    std::vector<std::string> joint_names_ = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTeleop>());
    rclcpp::shutdown();
    return 0;
}
