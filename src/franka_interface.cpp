// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka_interface.h"

volatile sig_atomic_t done = 0;
void signal_callback_handler(int signum)
{
    std::cout << "CTRL+C interrupted. " << std::endl;
    // Terminate program
    if (signum == SIGINT)
    {
        done = 1;
    }
    rclcpp::shutdown();
}

using namespace std::chrono_literals;

FrankaInterface::FrankaInterface(std::string robot_ip)
    : Node("franka_interface"), robot_ip_(robot_ip)
{
    subscriber_ = this->create_subscription<franka_streaming_driver::msg::FrankaJointCmd>("/franka_joint_commands", 1, std::bind(&FrankaInterface::command_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<franka_streaming_driver::msg::FrankaJointState>("/franka_joint_states", 1);
    timer_ = this->create_wall_timer(1ms, std::bind(&FrankaInterface::publish_state, this));

    libfranka_th_ = std::make_shared<std::thread>(&FrankaInterface::robot_run, this);
}

FrankaInterface::~FrankaInterface()
{
    libfranka_th_->join();
}

void FrankaInterface::robot_state_update(std::shared_ptr<franka::Model> model, const franka::RobotState &robot_state)
{
    coriolis_ = model->coriolis(robot_state);
    q_ = robot_state.q;
    dq_ = robot_state.dq;
    tau_J_ = robot_state.tau_J;
    tau_J_ext_ = robot_state.tau_ext_hat_filtered;

    const double kAlpha = 0.99; // approx 150 Hz low-pass filter
    const double kBeta = 0.314; // approx 50 Hz low-pass filter
    for (size_t i = 0; i < 7; i++)
    {
        dq_filtered_[i] = (1 - kAlpha) * dq_filtered_[i] + kAlpha * dq_[i];
        Kp_filtered_[i] = (1 - kBeta) * Kp_filtered_[i] + kBeta * Kp_[i];
        Kd_filtered_[i] = (1 - kBeta) * Kd_filtered_[i] + kBeta * Kd_[i];
    }

    // external input regulation
    dq_d_saturated_ = saturation(kMaxJointVelocity_, kMinJointVelocity_, dq_d_);
    dq_d_rate_limited_ = franka::limitRate(kMaxJointAcceleration_, dq_d_saturated_, dq_d_saturated_last_);

    q_d_saturated_ = saturation(kMaxJointPosition_, kMinJointPosition_, q_d_);
    q_d_rate_limited_ = franka::limitRate(kMaxJointVelocity_, q_d_saturated_, q_d_saturated_last_);

    tau_J_d_saturated_ = saturation(kMaxJointTorque_, kMinJointTorque_, tau_J_d_);
    tau_J_d_rate_limited_ = franka::limitRate(franka::kMaxTorqueRate, tau_J_d_saturated_, tau_J_d_saturated_last_);

    for (size_t i = 0; i < 7; i++)
    {
        dq_d_saturated_last_[i] = dq_d_saturated_[i];
        q_d_saturated_last_[i] = q_d_saturated_[i];
        tau_J_d_saturated_last_[i] = tau_J_d_saturated_[i];
    }
}

void FrankaInterface::robot_init()
{
    init_q_ = init_state_.q;
    for (size_t i = 0; i < 7; i++)
    {
        q_d_[i] = init_q_[i];
        q_d_saturated_last_[i] = init_q_[i];
    }
}

void FrankaInterface::robot_run()
{
    try
    {
        // Connect to robot.
        RCLCPP_INFO(this->get_logger(), "Connecting to the robot: %s", robot_ip_.c_str());
        franka::Robot robot(robot_ip_);
        robot.automaticErrorRecovery();
        setDefaultBehavior(robot);
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        // Load the kinematics and dynamics model.
        auto model = std::make_shared<franka::Model>(robot.loadModel());
        init_state_ = robot.readOnce();
        robot_init();

        // Define callback for the joint torque control loop.
        std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
            impedance_control_callback = [model, this](const franka::RobotState &robot_state, franka::Duration /*period*/) -> franka::Torques
        {
            /* ctrl+c finish running */
            if (done)
            {
                std::cout << std::endl
                          << "Ctrl + c entered, shutting down" << std::endl;
                return franka::MotionFinished(zero_torques_);
            }

            mutex_.lock();
            robot_state_update(model, robot_state);

            std::array<double, 7> tau_d_calculated;
            for (size_t i = 0; i < 7; i++)
            {
                tau_d_calculated[i] = Kp_filtered_[i] * (q_d_rate_limited_[i] - q_[i]) + Kd_filtered_[i] * (dq_d_rate_limited_[i] - dq_filtered_[i]) + tau_J_d_rate_limited_[i] + coriolis_[i];
                // tau_d_calculated[i] = 0.0;
            }

            std::array<double, 7> tau_d_saturated = saturation(kMaxJointTorque_, kMinJointTorque_, tau_d_calculated);
            std::array<double, 7> tau_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, tau_d_saturated, robot_state.tau_J_d);

            mutex_.unlock();

            // Send torque command.
            return tau_d_rate_limited;
        };

        // Start real-time control loop.
        robot.control(impedance_control_callback);
    }
    catch (const franka::Exception &ex)
    {
        std::cerr << ex.what() << std::endl;
    }
}

std::array<double, 7> FrankaInterface::saturation(const std::array<double, 7> max, const std::array<double, 7> min, const std::array<double, 7> input)
{
    std::array<double, 7> output;
    for (size_t i = 0; i < 7; i++)
    {
        output[i] = std::max(std::min(input[i], max[i]), min[i]);
    }
    return output;
}

void FrankaInterface::publish_state()
{
    msg_.name.resize(7);
    msg_.position.resize(7);
    msg_.velocity.resize(7);
    msg_.tau.resize(7);
    msg_.tau_ext.resize(7);
    msg_.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    for (size_t i = 0; i < 7; i++)
    {
        msg_.name[i] = joint_names_[i];
        msg_.position[i] = q_[i];
        msg_.velocity[i] = dq_[i];
        msg_.tau[i] = tau_J_[i];
        msg_.tau_ext[i] = tau_J_ext_[i];
    }
    publisher_->publish(msg_);
}

void FrankaInterface::command_callback(const franka_streaming_driver::msg::FrankaJointCmd::SharedPtr msg)
{
    std::copy(msg->position.begin(), msg->position.end(), q_d_.begin());
    std::copy(msg->velocity.begin(), msg->velocity.end(), dq_d_.begin());
    std::copy(msg->effort.begin(), msg->effort.end(), tau_J_d_.begin());
    std::copy(msg->kp.begin(), msg->kp.end(), Kp_.begin());
    std::copy(msg->kd.begin(), msg->kd.end(), Kd_.begin());
}

int main(int argc, char **argv)
{

    signal(SIGINT, signal_callback_handler);

    // Check whether the required arguments were passed.
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    rclcpp::InitOptions option = rclcpp::InitOptions();
    option.shutdown_on_signal = false;
    rclcpp::init(argc, argv, option);
    auto franka_interface = std::make_shared<FrankaInterface>(argv[1]);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(franka_interface);
    executor.spin();

    std::cout << "Control Finished." << std::endl;

    return 0;
}