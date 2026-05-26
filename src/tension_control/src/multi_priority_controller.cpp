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

#include <algorithm>
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

  auto log_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/log_state", rclcpp::SystemDefaultsQoS());
  log_pub_ = std::make_shared<
      realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(log_publisher);
  // Pre-allocate so update() never resizes from RT context.
  log_pub_->msg_.data.assign(7, 0.0);

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured. anchor=[%.3f, %.3f, %.3f], T_des=%.1f N",
              anchor_position_.x(), anchor_position_.y(), anchor_position_.z(), desired_tension_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiPriorityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  // Defer pose initialization to the first update() call to avoid racing with the RT thread.
  pose_initialized_ = false;
  tension_initialized_ = false;
  dq_filtered_.setZero();
  tension_prev_ = 0.0;
  tension_filt_ = 0.0;
  tension_deriv_filt_ = 0.0;
  fc_prev_ = 0.0;
  desired_tension_initial_ = 0.0;
  setpoint_ramp_ = 0.0;
  tau_prev_.setZero();
  gain_ramp_ = 0.0;
  F_ext_offset_.setZero();
  log_decim_counter_ = 0;

  RCLCPP_INFO(get_node()->get_logger(), "Activated — pose will be initialized on first update.");
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
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  updateJointStates();

  const double dt = period.seconds();
  gain_ramp_ = std::min(1.0, gain_ramp_ + dt / kGainRampDuration);

  // ~10 ms time constant: smooths encoder differentiation noise without delaying
  // the damping torque. kAlpha=0.99 gave ~100 ms, far too slow to oppose a runaway.
  const double kAlpha = 0.9;
  dq_filtered_ = (1.0 - kAlpha) * dq_ + kAlpha * dq_filtered_;

  // ── Robot model quantities ──────────────────────────────────────────────────
  // NOTE: franka_hardware + libfranka 0.13 ActiveControl applies gravity/coriolis
  // compensation internally. We only command ADDITIONAL torques on top of that.
  auto jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  auto pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  auto tau_ext_array = franka_robot_model_->getJointExternalTorque();

  // Zero Jacobian layout: 6×7 column-major. Rows 0-2: linear, rows 3-5: angular.
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

  Eigen::Map<const Eigen::Matrix4d> T_ee(pose_array.data());
  const Eigen::Vector3d p_ee = T_ee.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_ee = T_ee.block<3, 3>(0, 0);

  // Initialize desired pose from current state on the first valid update cycle.
  if (!pose_initialized_) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    p_des_ = p_ee;
    R_des_ = R_ee;
    p_des_rt_ = p_ee;
    R_des_rt_ = R_ee;
    pose_initialized_ = true;
    RCLCPP_INFO(get_node()->get_logger(),
                "Pose initialized. EE pos=[%.3f, %.3f, %.3f]",
                p_ee.x(), p_ee.y(), p_ee.z());
  }

  // Recover the Cartesian external force from the joint-space residual:
  //   tau_ext ≈ J_linᵀ · F_ext   (ignoring external moments at the EE)
  //   F_ext   ≈ (J_lin · J_linᵀ + λI)⁻¹ · J_lin · tau_ext   (damped least-squares)
  // O_F_ext_hat_K reads zero on this rig despite tau_ext_hat_filtered being alive,
  // so we project the joint-space estimate ourselves.
  const Eigen::Map<const Vector7d> tau_ext(tau_ext_array.data());
  const auto J_lin = J.topRows(3);
  const Eigen::Matrix3d JJT =
      J_lin * J_lin.transpose() + 1e-6 * Eigen::Matrix3d::Identity();
  const Eigen::Vector3d F_ext_raw = JJT.ldlt().solve(J_lin * tau_ext);

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

    // First valid cycle: tare the Cartesian-force estimate so the controller sees
    // ΔF from this moment onward. Recompute the corrected F_ext locally so this
    // cycle's tension reading is also zero (instead of using the pre-tare value).
    if (!tension_initialized_) {
      F_ext_offset_ = F_ext_raw;
    }
    const Eigen::Vector3d F_ext_corrected = F_ext_raw - F_ext_offset_;
    // Cable tension is a magnitude — a real cable can only pull, never push, so the
    // sign of the projection along cable_dir carries no physical meaning here.
    // Using |projection| lets a measurement of 8 N produce error = 10 − 8 = 2 N
    // regardless of which side of the tare baseline the residual landed on.
    // The fc direction logic below still reduces to "push EE away from anchor when
    // error > 0", because increasing |EE − anchor| stretches the cable, which is
    // the one and only way to raise tension.
    const double tension_measured_raw = std::abs(cable_dir.dot(F_ext_corrected));

    // Low-pass the tension measurement before it enters the proportional path.
    // F_ext is reconstructed from tau_ext_hat by least-squares, and that residual
    // carries several N of broadband noise. Without smoothing here, Kp · noise
    // produces visible end-effector vibration around the desired pose. ~50 ms
    // time constant: short enough to follow real cable dynamics (which evolve on
    // hundreds of ms), long enough to suppress per-cycle spikes from the residual.
    constexpr double kTensionAlpha = 0.98;  // ~50 ms TC at 1 kHz
    if (!tension_initialized_) {
      tension_filt_ = tension_measured_raw;
    } else {
      tension_filt_ = (1.0 - kTensionAlpha) * tension_measured_raw + kTensionAlpha * tension_filt_;
    }
    const double tension_measured = tension_filt_;

    // Seed derivative state and capture the starting setpoint on the first cycle.
    // After taring, tension_measured is exactly 0 — the setpoint ramp goes from
    // 0 → desired_tension_ over kSetpointRampDuration.
    if (!tension_initialized_) {
      tension_prev_ = tension_measured;
      desired_tension_initial_ = tension_measured;
      tension_initialized_ = true;
      RCLCPP_INFO(get_node()->get_logger(),
                  "Tared. F_ext_offset=[%.3f, %.3f, %.3f] N  cable_dir=[%.3f, %.3f, %.3f]  "
                  "cable_len=%.3f m  -> ramping tension setpoint from 0 to %.3f N over %.1f s.",
                  F_ext_offset_.x(), F_ext_offset_.y(), F_ext_offset_.z(),
                  cable_dir.x(), cable_dir.y(), cable_dir.z(), cable_length,
                  desired_tension_, kSetpointRampDuration);
    }

    // Advance the setpoint ramp and compute the active setpoint.
    setpoint_ramp_ = std::min(1.0, setpoint_ramp_ + dt / kSetpointRampDuration);
    const double desired_tension_rt =
        desired_tension_initial_ + setpoint_ramp_ * (desired_tension_ - desired_tension_initial_);

    const double tension_error = desired_tension_rt - tension_measured;
    const double tension_deriv_raw = (dt > 1e-9) ? (tension_measured - tension_prev_) / dt : 0.0;
    tension_prev_ = tension_measured;

    // Low-pass the derivative — O_F_ext_hat_K is noisy and Kd amplifies that noise by 1/dt.
    // ~20 ms time constant; enough to suppress per-cycle spikes without killing damping.
    constexpr double kDerivAlpha = 0.95;
    tension_deriv_filt_ =
        (1.0 - kDerivAlpha) * tension_deriv_raw + kDerivAlpha * tension_deriv_filt_;

    // Deadband: suppress tension control for small errors (F_ext noise without cable).
    constexpr double kTensionDeadband = 0.5;  // [N]
    const double tension_error_db =
        (std::abs(tension_error) > kTensionDeadband)
            ? tension_error - std::copysign(kTensionDeadband, tension_error)
            : 0.0;

    // Negative sign: to increase tension, EE must move AWAY from anchor (stretching the cable).
    // fc > 0 would push EE toward anchor (shortening cable, reducing tension) — wrong direction.
    // gain_ramp_ scales the whole term so a pre-tensioned cable does not produce a step torque
    // at activation (the original cause of immediate reflex aborts).
    double fc = -gain_ramp_ * (tension_kp_ * tension_error_db - tension_kd_ * tension_deriv_filt_);

    // Rate-limit fc. The cable has a one-sided constraint (T ≥ 0): when the EE
    // is closer than the cable's natural length the cable goes slack and T → 0,
    // and beyond that distance T grows with extension. The slack↔taut boundary
    // is non-smooth, so a fast-changing fc can drive the EE across it
    // repeatedly, producing a limit cycle (slack → drop in T → push hard →
    // overshoot → taut → spike in T → pull back → slack …). 50 N/s lets fc
    // reach saturation in ~0.4 s — slow enough that the EE physically moves
    // along cable_dir without slamming the boundary, fast enough to track the
    // setpoint within a few seconds.
    constexpr double kMaxFcRate = 50.0;          // [N/s]
    const double max_dfc = kMaxFcRate * dt;       // per-cycle cap
    fc = std::clamp(fc - fc_prev_, -max_dfc, max_dfc) + fc_prev_;

    // Hard saturation on the cable force command — last-resort safety against any large
    // residual error (e.g. operator perturbation, noise burst). Keeps joint torques bounded
    // even if the user tunes Kp aggressively.
    constexpr double kMaxCableForce = 20.0;  // [N]
    fc = std::clamp(fc, -kMaxCableForce, kMaxCableForce);
    fc_prev_ = fc;

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

  // Position error clamped to prevent large movements
  Eigen::Vector3d ep = p_des_rt_ - p_ee;
  const double kMaxPosError = 0.05; // 5 cm maximum position error
  if (ep.norm() > kMaxPosError) {
    ep = ep.normalized() * kMaxPosError;
  }

  // Orientation error: small-angle formula from R_err = R_des * R_curr^T
  // Valid for errors up to ~90°; sufficient for typical trajectory tracking.
  const Eigen::Matrix3d R_err = R_des_rt_ * R_ee.transpose();
  Eigen::Vector3d eo;
  eo << R_err(2, 1) - R_err(1, 2),
        R_err(0, 2) - R_err(2, 0),
        R_err(1, 0) - R_err(0, 1);
  eo *= 0.5;

  // Clamp orientation error to prevent large rotations (max ~30 degrees initially)
  const double kMaxAngError = 0.5; // ~28.6 degrees in radians
  if (eo.norm() > kMaxAngError) {
    eo = eo.normalized() * kMaxAngError;
  }

  const Eigen::Vector3d v_ee = J.topRows(3) * dq_filtered_;
  const Eigen::Vector3d omega_ee = J.bottomRows(3) * dq_filtered_;

  Eigen::Matrix<double, 6, 1> Fe;
  // gain_ramp_ scales proportional terms so the secondary task ramps in alongside Task 1.
  // Derivative terms are not ramped — they should always damp motion.
  Fe << gain_ramp_ * ee_kp_ * ep - ee_kd_ * v_ee,
        gain_ramp_ * orientation_kp_ * eo - orientation_kd_ * omega_ee;

  const Vector7d tau_ee = N * J.transpose() * Fe;

  // ── Total torque ────────────────────────────────────────────────────────────
  // Gravity and Coriolis are compensated internally by franka_hardware (libfranka 0.13).
  //
  // Order matters here:
  //   1. Clamp the smooth control torque to joint limits.
  //   2. Rate-limit the smooth control torque (anti-jerk).
  //   3. Add velocity damping AFTER the rate limiter so it can react instantly to
  //      sudden motion (a rate-limited damping cannot oppose a runaway in time).
  //   4. Final clamp to joint limits (damping + control may exceed them together).
  //
  // The previous version added damping before the rate limiter and again after, which
  // double-counted it and bypassed both the rate limiter and the torque clamp.
  Vector7d tau_control = tau_cable + tau_ee;

  const Vector7d tau_limit = (Vector7d() << 87, 87, 87, 87, 12, 12, 12).finished();
  tau_control = tau_control.cwiseMax(-tau_limit).cwiseMin(tau_limit);

  constexpr double kDeltaTauMax = 0.5;  // [Nm] per 1 ms cycle (= 500 Nm/s)
  tau_control =
      (tau_control - tau_prev_).cwiseMax(-kDeltaTauMax).cwiseMin(kDeltaTauMax) + tau_prev_;
  tau_prev_ = tau_control;

  // Joint-velocity damping. With the tension LPF in place, the proportional
  // path lags by ~50 ms, so saturated cable forces persist briefly in one
  // direction. Without enough joint damping the integrated velocity exceeds
  // libfranka's reflex threshold (~2 rad/s on shoulder joints). 6 Nm·s/rad
  // gives a stable margin: at 1 rad/s the damping torque already cancels a
  // saturated commanded torque.
  constexpr double kVelDamping = 6.0;
  const Vector7d tau_damping = -kVelDamping * dq_filtered_;
  Vector7d tau = (tau_control + tau_damping).cwiseMax(-tau_limit).cwiseMin(tau_limit);

  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau(i));
  }

  // Diagnostics (throttled to 1 Hz). T_des_rt is the active (ramped) setpoint; the
  // ramp factors expose how much of each safety ramp is currently active.
  const double T_des_rt =
      desired_tension_initial_ + setpoint_ramp_ * (desired_tension_ - desired_tension_initial_);
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "cable_len=%.3fm  T_meas=%.2fN  T_des_rt=%.2fN  T_des=%.2fN  "
                       "gain_ramp=%.2f  setpoint_ramp=%.2f  "
                       "F_ext_raw=[%.2f,%.2f,%.2f]N  F_offset=[%.2f,%.2f,%.2f]N  "
                       "ee_err=[%.3f, %.3f, %.3f]m  tau=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
                       cable_length, tension_prev_, T_des_rt, desired_tension_,
                       gain_ramp_, setpoint_ramp_,
                       F_ext_raw.x(), F_ext_raw.y(), F_ext_raw.z(),
                       F_ext_offset_.x(), F_ext_offset_.y(), F_ext_offset_.z(),
                       ep.x(), ep.y(), ep.z(),
                       tau(0), tau(1), tau(2), tau(3), tau(4), tau(5), tau(6));

  // ── Diagnostic log publisher (100 Hz, RT-safe) ─────────────────────────────
  if (++log_decim_counter_ >= kLogDecimation) {
    log_decim_counter_ = 0;
    if (log_pub_ && log_pub_->trylock()) {
      auto& data = log_pub_->msg_.data;
      data[0] = time.seconds();
      data[1] = desired_tension_;   // target setpoint (parameter)
      data[2] = T_des_rt;            // active setpoint (ramped)
      data[3] = tension_filt_;       // filtered measured tension
      data[4] = p_ee.x();
      data[5] = p_ee.y();
      data[6] = p_ee.z();
      log_pub_->unlockAndPublish();
    }
  }

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
