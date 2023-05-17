// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <memory>
#include <string>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <rclcpp/rclcpp.hpp>
#include "franka_streaming_driver/msg/franka_joint_state.hpp"
#include "franka_streaming_driver/msg/franka_joint_cmd.hpp"

#include "examples_common.h"

class FrankaInterface : public rclcpp::Node
{
public:
    FrankaInterface(std::string robot_ip);
    ~FrankaInterface();
    void robot_run();

private:
    void robot_init();
    void robot_state_update(std::shared_ptr<franka::Model> model, const franka::RobotState &robot_state);
    void publish_state();
    void command_callback(const franka_streaming_driver::msg::FrankaJointCmd::SharedPtr msg);
    std::array<double, 7> saturation(std::array<double, 7> max, std::array<double, 7> min, std::array<double, 7> input);

    rclcpp::Publisher<franka_streaming_driver::msg::FrankaJointState>::SharedPtr publisher_;
    rclcpp::Subscription<franka_streaming_driver::msg::FrankaJointCmd>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    const std::string robot_ip_;
    // robot state
    franka::RobotState init_state_;
    franka::Torques zero_torques_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    // variables storing the robot state, all default zero
    std::array<double, 7> coriolis_;
    std::array<double, 7> q_;
    std::array<double, 7> dq_;
    std::array<double, 7> dq_filtered_;
    std::array<double, 7> tau_J_;
    std::array<double, 7> tau_J_ext_;

    std::array<double, 7> init_q_;

    std::array<double, 7> q_d_;
    std::array<double, 7> dq_d_;
    std::array<double, 7> tau_J_d_;
    std::array<double, 7> q_d_saturated_;
    std::array<double, 7> q_d_rate_limited_;
    std::array<double, 7> q_d_saturated_last_;
    std::array<double, 7> dq_d_saturated_;
    std::array<double, 7> dq_d_rate_limited_;
    std::array<double, 7> dq_d_saturated_last_;
    std::array<double, 7> tau_J_d_saturated_;
    std::array<double, 7> tau_J_d_rate_limited_;
    std::array<double, 7> tau_J_d_saturated_last_;

    rclcpp::Time start_time_;
    franka_streaming_driver::msg::FrankaJointState msg_;
    std::vector<std::string> joint_names_ = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

    std::mutex mutex_;
    std::shared_ptr<std::thread> libfranka_th_;

    // variables for impedance gains
    // const std::array<double, 7> Kp_ = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // const std::array<double, 7> Kd_ = {{50.0, 50.0, 50.0, 50.0, 20.0, 20.0, 15.0}};
    std::array<double, 7> Kp_;
    std::array<double, 7> Kd_;
    std::array<double, 7> Kp_filtered_;
    std::array<double, 7> Kd_filtered_;

    const double kLimitEps_ = franka::kLimitEps;
    const double kDeltaT_ = franka::kDeltaT;
    const double kTolNumberPacketsLost_ = franka::kTolNumberPacketsLost;
    const std::array<double, 7> kMaxJointAcceleration_ = franka::kMaxJointAcceleration;
    const std::array<double, 7> kMaxJointVelocity_ = franka::kMaxJointVelocity;
    const std::array<double, 7> kMinJointVelocity_ = {
        {-2.1750 + kLimitEps_ + kTolNumberPacketsLost_ * kDeltaT_ * kMaxJointAcceleration_[0],
         -2.1750 + kLimitEps_ + kTolNumberPacketsLost_ *kDeltaT_ *kMaxJointAcceleration_[1],
         -2.1750 + kLimitEps_ + kTolNumberPacketsLost_ *kDeltaT_ *kMaxJointAcceleration_[2],
         -2.1750 + kLimitEps_ + kTolNumberPacketsLost_ *kDeltaT_ *kMaxJointAcceleration_[3],
         -2.6100 + kLimitEps_ + kTolNumberPacketsLost_ *kDeltaT_ *kMaxJointAcceleration_[4],
         -2.6100 + kLimitEps_ + kTolNumberPacketsLost_ *kDeltaT_ *kMaxJointAcceleration_[5],
         -2.6100 + kLimitEps_ + kTolNumberPacketsLost_ *kDeltaT_ *kMaxJointAcceleration_[6]}};
    const std::array<double, 7> kMaxJointTorque_ = {{87.0 - kLimitEps_, 87.0 - kLimitEps_, 87.0 - kLimitEps_, 87.0 - kLimitEps_,
                                                     12.0 - kLimitEps_, 12.0 - kLimitEps_, 12.0 - kLimitEps_}};
    const std::array<double, 7> kMinJointTorque_ = {{-87.0 + kLimitEps_, -87.0 + kLimitEps_, -87.0 + kLimitEps_, -87.0 + kLimitEps_,
                                                     -12.0 + kLimitEps_, -12.0 + kLimitEps_, -12.0 + kLimitEps_}};
    const std::array<double, 7> kMaxJointPosition_ = {{2.7437 - kLimitEps_, 1.7837 - kLimitEps_, 2.9007 - kLimitEps_, -0.1518 - kLimitEps_,
                                                       2.8065 - kLimitEps_, 4.5169 - kLimitEps_, 3.0159 - kLimitEps_}};
    const std::array<double, 7> kMinJointPosition_ = {{-2.7437 + kLimitEps_, -1.7837 + kLimitEps_, -2.9007 + kLimitEps_, -3.0421 + kLimitEps_,
                                                       -2.8065 + kLimitEps_, 0.5445 + kLimitEps_, -3.0159 + kLimitEps_}};
};
