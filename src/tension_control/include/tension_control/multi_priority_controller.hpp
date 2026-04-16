// Copyright 2024 Eduardo
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka/robot_state.h"
#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace tension_control {

// Extends FrankaRobotModel to expose the Franka-estimated external wrench
// (O_F_ext_hat_K: external forces/torques at end-effector expressed in world frame).
class FrankaRobotModelWithState : public franka_semantic_components::FrankaRobotModel {
 public:
  using FrankaRobotModel::FrankaRobotModel;

  std::array<double, 6> getExternalWrench() {
    if (!initialized) {
      initialize();
    }
    return {robot_state->O_F_ext_hat_K[0], robot_state->O_F_ext_hat_K[1],
            robot_state->O_F_ext_hat_K[2], robot_state->O_F_ext_hat_K[3],
            robot_state->O_F_ext_hat_K[4], robot_state->O_F_ext_hat_K[5]};
  }
};

/**
 * Multi-priority torque controller for cable manipulation.
 *
 * Task 1 (primary): Maintain a desired cable tension. The cable connects the
 *   end-effector to a fixed anchor point in the world frame.
 *
 * Task 2 (secondary): Track a desired end-effector pose in the null-space of
 *   Task 1, so that it never perturbs cable tension.
 *
 * Control law:
 *   τ = τ_grav + τ_coriolis + Jcᵀ fc + (I - Jc⁺Jc) Jeᵀ Fe
 *
 * The desired EE pose is received on ~/desired_ee_pose (geometry_msgs/PoseStamped).
 * On activation, the current EE pose is used as the initial setpoint.
 */
class MultiPriorityController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  static constexpr int kNumJoints = 7;

  std::string arm_id_;
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  std::unique_ptr<FrankaRobotModelWithState> franka_robot_model_;

  // Joint state
  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;  // low-pass filtered velocities for damping terms

  // Task 1: Cable tension control
  Eigen::Vector3d anchor_position_;  // fixed anchor point in world frame [m]
  double desired_tension_{0.0};      // [N]
  double tension_kp_{0.0};
  double tension_kd_{0.0};
  double tension_prev_{0.0};  // tension from previous step (for derivative)

  // Task 2: EE pose control
  double ee_kp_{0.0};
  double ee_kd_{0.0};
  double orientation_kp_{0.0};
  double orientation_kd_{0.0};

  // Desired EE pose — written by subscription callback (non-RT), read in update() (RT).
  // Protected by pose_mutex_ on the write side. The RT thread uses try_lock so it
  // never blocks; if the lock is unavailable it keeps the cached pose.
  Eigen::Vector3d p_des_;
  Eigen::Matrix3d R_des_;
  std::mutex pose_mutex_;

  // RT-local cache: only accessed inside update(), never from the subscription callback.
  Eigen::Vector3d p_des_rt_;
  Eigen::Matrix3d R_des_rt_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  void updateJointStates();
};

}  // namespace tension_control
