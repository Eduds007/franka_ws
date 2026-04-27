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

  // libfranka's joint-space external-torque estimate (gravity/coriolis-compensated,
  // low-pass filtered). On this rig O_F_ext_hat_K is reported as identically zero,
  // but tau_ext_hat_filtered is alive — the controller recovers Cartesian force from
  // it by least-squares through the Jacobian.
  std::array<double, 7> getJointExternalTorque() {
    if (!initialized) {
      initialize();
    }
    return {robot_state->tau_ext_hat_filtered[0], robot_state->tau_ext_hat_filtered[1],
            robot_state->tau_ext_hat_filtered[2], robot_state->tau_ext_hat_filtered[3],
            robot_state->tau_ext_hat_filtered[4], robot_state->tau_ext_hat_filtered[5],
            robot_state->tau_ext_hat_filtered[6]};
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
  double desired_tension_{0.0};      // target setpoint from parameter [N]
  double tension_kp_{0.0};
  double tension_kd_{0.0};
  double tension_prev_{0.0};       // tension from previous step (for derivative)
  double tension_deriv_filt_{0.0}; // low-pass filtered tension derivative (F_ext is noisy)

  // Setpoint ramp — captures the tension at activation and linearly ramps the
  // active setpoint from there to desired_tension_ over kSetpointRampDuration
  // seconds. Combined with gain_ramp_, this keeps the tension error ~0 during
  // start-up so a pre-tensioned cable does not produce a step torque even when
  // the gain is partially up.
  double desired_tension_initial_{0.0};               // captured at first valid cycle [N]
  double setpoint_ramp_{0.0};                         // 0 → 1 over kSetpointRampDuration
  static constexpr double kSetpointRampDuration = 5.0; // [s]

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

  // Torque rate limiter — previous commanded torque, for 1 Nm/ms ramp limit.
  Vector7d tau_prev_;

  // Gain ramp — linearly scales all gains from 0→1 over kGainRampDuration seconds.
  double gain_ramp_{0.0};
  static constexpr double kGainRampDuration = 3.0;  // [s]

  // Flag: desired pose not yet initialized from robot state (done in first update()).
  bool pose_initialized_{false};

  // Flag: tension_prev_ not yet seeded from first measured tension (prevents derivative spike).
  bool tension_initialized_{false};

  // Cartesian-force tare. The joint-space residual tau_ext_hat_filtered has a baseline
  // offset from gravity/payload mismodel and friction, so the projected F_ext shows
  // several N of phantom force at rest. We capture F_ext on the first valid cycle and
  // subtract it on every subsequent cycle, so the controller sees ΔF from activation.
  Eigen::Vector3d F_ext_offset_{Eigen::Vector3d::Zero()};

  // RT-local cache: only accessed inside update(), never from the subscription callback.
  Eigen::Vector3d p_des_rt_;
  Eigen::Matrix3d R_des_rt_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  void updateJointStates();
};

}  // namespace tension_control
