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

#include "tension_control/multi_priority_controller.hpp"

#include <cassert>
#include <stdexcept>

#include "franka/model.h"

namespace tension_control {

// ──────────────────────────────────────────────────────────────────────────────
// Interface configuration
// ──────────────────────────────────────────────────────────────────────────────

controller_interface::InterfaceConfiguration
MultiPriorityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
MultiPriorityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Joint positions and velocities (used to read q, dq)
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  // Franka model and state (Jacobian, gravity, Coriolis, external wrench)
  for (const auto& name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(name);
  }
  return config;
}

// ──────────────────────────────────────────────────────────────────────────────
// Lifecycle callbacks
// ──────────────────────────────────────────────────────────────────────────────

CallbackReturn MultiPriorityController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("anchor_position", {});
    auto_declare<double>("desired_tension", 5.0);
    auto_declare<double>("tension_kp", 50.0);
    auto_declare<double>("tension_kd", 2.0);
    auto_declare<double>("ee_kp", 200.0);
    auto_declare<double>("ee_kd", 20.0);
    auto_declare<double>("orientation_kp", 10.0);
    auto_declare<double>("orientation_kd", 1.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiPriorityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  auto anchor = get_node()->get_parameter("anchor_position").as_double_array();
  if (anchor.size() != 3) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "anchor_position must have exactly 3 elements [x, y, z], got %zu", anchor.size());
    return CallbackReturn::FAILURE;
  }
  anchor_position_ = Eigen::Vector3d(anchor[0], anchor[1], anchor[2]);

  desired_tension_ = get_node()->get_parameter("desired_tension").as_double();
  tension_kp_ = get_node()->get_parameter("tension_kp").as_double();
  tension_kd_ = get_node()->get_parameter("tension_kd").as_double();
  ee_kp_ = get_node()->get_parameter("ee_kp").as_double();
  ee_kd_ = get_node()->get_parameter("ee_kd").as_double();
  orientation_kp_ = get_node()->get_parameter("orientation_kp").as_double();
  orientation_kd_ = get_node()->get_parameter("orientation_kd").as_double();

  franka_robot_model_ = std::make_unique<FrankaRobotModelWithState>(
      arm_id_ + "/" + k_robot_model_interface_name,
      arm_id_ + "/" + k_robot_state_interface_name);

  // Subscribe to desired EE pose. Non-RT; only updates the shared pose buffer.
  pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/desired_ee_pose", 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        Eigen::Vector3d p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        const auto& q = msg->pose.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        quat.normalize();
        std::lock_guard<std::mutex> lock(pose_mutex_);
        p_des_ = p;
        R_des_ = quat.toRotationMatrix();
      });

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured. anchor=[%.3f, %.3f, %.3f], T_des=%.1f N",
              anchor_position_.x(), anchor_position_.y(), anchor_position_.z(), desired_tension_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiPriorityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  // Read current EE pose and use it as the initial desired pose.
  auto pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Map<const Eigen::Matrix4d> T_ee(pose_array.data());

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    p_des_ = T_ee.block<3, 1>(0, 3);
    R_des_ = T_ee.block<3, 3>(0, 0);
  }
  p_des_rt_ = p_des_;
  R_des_rt_ = R_des_;

  updateJointStates();
  dq_filtered_.setZero();
  tension_prev_ = 0.0;

  RCLCPP_INFO(get_node()->get_logger(),
              "Activated. Initial EE pos=[%.3f, %.3f, %.3f]",
              p_des_rt_.x(), p_des_rt_.y(), p_des_rt_.z());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiPriorityController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────────────────────
// Control update (1 kHz)
// ──────────────────────────────────────────────────────────────────────────────

controller_interface::return_type MultiPriorityController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  updateJointStates();

  const double dt = period.seconds();
  const double kAlpha = 0.99;
  dq_filtered_ = (1.0 - kAlpha) * dq_ + kAlpha * dq_filtered_;

  // ── Robot model quantities ──────────────────────────────────────────────────
  auto jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  auto gravity_array = franka_robot_model_->getGravityForceVector();
  auto coriolis_array = franka_robot_model_->getCoriolisForceVector();
  auto pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  auto f_ext_array = franka_robot_model_->getExternalWrench();

  // Zero Jacobian layout: 6×7 column-major. Rows 0-2: linear, rows 3-5: angular.
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());
  Eigen::Map<const Vector7d> tau_grav(gravity_array.data());
  Eigen::Map<const Vector7d> tau_cor(coriolis_array.data());

  Eigen::Map<const Eigen::Matrix4d> T_ee(pose_array.data());
  const Eigen::Vector3d p_ee = T_ee.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_ee = T_ee.block<3, 3>(0, 0);

  const Eigen::Vector3d F_ext(f_ext_array[0], f_ext_array[1], f_ext_array[2]);

  // ── Task 1: Cable tension (primary) ────────────────────────────────────────
  //
  //   Jc = cable_dir^T · J_lin   (1×7)
  //   fc = Kp·(T_des − T_meas) − Kd·dT/dt   (scalar)
  //   τ₁ = Jcᵀ · fc

  const Eigen::Vector3d cable_vec = anchor_position_ - p_ee;
  const double cable_length = cable_vec.norm();

  Vector7d tau_cable = Vector7d::Zero();
  Eigen::Matrix<double, 1, 7> Jc = Eigen::Matrix<double, 1, 7>::Zero();
  Eigen::Matrix<double, 7, 1> Jc_pseudo_inv = Eigen::Matrix<double, 7, 1>::Zero();

  if (cable_length > 1e-3) {
    const Eigen::Vector3d cable_dir = cable_vec / cable_length;

    Jc = cable_dir.transpose() * J.topRows(3);

    const double tension_measured = cable_dir.dot(F_ext);
    const double tension_error = desired_tension_ - tension_measured;
    const double tension_deriv = (dt > 1e-9) ? (tension_measured - tension_prev_) / dt : 0.0;
    tension_prev_ = tension_measured;

    const double fc = tension_kp_ * tension_error - tension_kd_ * tension_deriv;
    tau_cable = Jc.transpose() * fc;

    const double Jc_sq = (Jc * Jc.transpose())(0, 0);
    if (Jc_sq > 1e-8) {
      Jc_pseudo_inv = Jc.transpose() / Jc_sq;
    }
  }

  // ── Null-space projector: N = I − Jc⁺·Jc ────────────────────────────────
  const Matrix7d N = Matrix7d::Identity() - Jc_pseudo_inv * Jc;

  // ── Task 2: EE pose tracking in null-space ─────────────────────────────────
  //
  //   Fe = [Kp·ep − Kd·v_ee ; Ko·eo − Kdo·ω_ee]   (6×1)
  //   τ₂ = N · Jeᵀ · Fe

  // Pull latest desired pose (non-blocking).
  {
    std::unique_lock<std::mutex> lock(pose_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      p_des_rt_ = p_des_;
      R_des_rt_ = R_des_;
    }
    // If lock unavailable, keep p_des_rt_ / R_des_rt_ from previous cycle.
  }

  const Eigen::Vector3d ep = p_des_rt_ - p_ee;

  // Orientation error: small-angle formula from R_err = R_des * R_curr^T
  // Valid for errors up to ~90°; sufficient for typical trajectory tracking.
  const Eigen::Matrix3d R_err = R_des_rt_ * R_ee.transpose();
  Eigen::Vector3d eo;
  eo << R_err(2, 1) - R_err(1, 2),
        R_err(0, 2) - R_err(2, 0),
        R_err(1, 0) - R_err(0, 1);
  eo *= 0.5;

  const Eigen::Vector3d v_ee = J.topRows(3) * dq_filtered_;
  const Eigen::Vector3d omega_ee = J.bottomRows(3) * dq_filtered_;

  Eigen::Matrix<double, 6, 1> Fe;
  Fe << ee_kp_ * ep - ee_kd_ * v_ee,
        orientation_kp_ * eo - orientation_kd_ * omega_ee;

  const Vector7d tau_ee = N * J.transpose() * Fe;

  // ── Total torque ────────────────────────────────────────────────────────────
  Vector7d tau = tau_grav + tau_cor + tau_cable + tau_ee;

  // Clamp to Franka joint torque limits (joints 1-4: 87 Nm, 5-7: 12 Nm).
  const Vector7d tau_limit = (Vector7d() << 87, 87, 87, 87, 12, 12, 12).finished();
  tau = tau.cwiseMax(-tau_limit).cwiseMin(tau_limit);

  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau(i));
  }

  // Diagnostics (throttled to 1 Hz)
  RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                        "cable_len=%.3fm  T_meas=%.2fN  T_des=%.2fN  "
                        "ee_err=[%.3f, %.3f, %.3f]m",
                        cable_length, tension_prev_, desired_tension_,
                        ep.x(), ep.y(), ep.z());

  return controller_interface::return_type::OK;
}

// ──────────────────────────────────────────────────────────────────────────────
// Private helpers
// ──────────────────────────────────────────────────────────────────────────────

void MultiPriorityController::updateJointStates() {
  // state_interfaces_ layout (set in state_interface_configuration):
  //   [0, 2, 4, ..., 12] = joint 1-7 positions
  //   [1, 3, 5, ..., 13] = joint 1-7 velocities
  //   [14, 15]           = franka robot_model, robot_state (managed by semantic component)
  for (int i = 0; i < kNumJoints; ++i) {
    const auto& pos_iface = state_interfaces_.at(static_cast<size_t>(2 * i));
    const auto& vel_iface = state_interfaces_.at(static_cast<size_t>(2 * i + 1));
    assert(pos_iface.get_interface_name() == "position");
    assert(vel_iface.get_interface_name() == "velocity");
    q_(i) = pos_iface.get_value();
    dq_(i) = vel_iface.get_value();
  }
}

}  // namespace tension_control

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(tension_control::MultiPriorityController,
                       controller_interface::ControllerInterface)
