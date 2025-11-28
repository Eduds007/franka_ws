#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define M_PI_4 0.78539816339744830962 /* pi/4 */

class MoveHomingNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  MoveHomingNode(bool use_cable_position = false) : Node("move_homing_node"), use_cable_position_(use_cable_position)
  {
    // Create action client for joint trajectory controller
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/joint_trajectory_controller/follow_joint_trajectory");

    // Wait for action server to be available
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Action server available, sending homing trajectory");
    send_homing_trajectory();
  }

private:
  void send_homing_trajectory()
  {
    auto goal_msg = FollowJointTrajectory::Goal();
    
    // Set joint names for panda robot
    goal_msg.trajectory.joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"
    };

    // Create trajectory point for homing position
    trajectory_msgs::msg::JointTrajectoryPoint point;
    
    if (use_cable_position_) {
      // Get to cable position
      
      point.positions = {0.025, 0.741, 0.01, -1.43, -0.05, 2.17, -0.73};
      RCLCPP_INFO(this->get_logger(), "Using CABLE position");
    } else {
      // Go Homing
      point.positions = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
      RCLCPP_INFO(this->get_logger(), "Using CURRENT position");
    }
    
    point.velocities = {0, 0, 0, 0, 0, 0, 0};
    point.time_from_start = rclcpp::Duration::from_seconds(5.0);

    goal_msg.trajectory.points.push_back(point);

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      [this](const GoalHandleFJT::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.feedback_callback =
      [this](GoalHandleFJT::SharedPtr, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
        // Log progress if needed
        (void)feedback; // Suppress unused parameter warning
      };

    send_goal_options.result_callback =
      [this](const GoalHandleFJT::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Homing procedure finished successfully");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Homing procedure was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Homing procedure was canceled");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
        rclcpp::shutdown();
      };

    RCLCPP_INFO(this->get_logger(), "Sending homing trajectory goal");
    trajectory_client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  bool use_cable_position_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  // Check command line arguments
  bool use_cable_position = false;
  if (argc > 1) {
    std::string arg = argv[1];
    if (arg == "cable" || arg == "--cable") {
      use_cable_position = true;
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("move_homing"), "Starting Franka robot homing procedure");
  RCLCPP_INFO(rclcpp::get_logger("move_homing"), "Position mode: %s", 
              use_cable_position ? "CABLE" : "CURRENT");
  RCLCPP_INFO(rclcpp::get_logger("move_homing"), "WARNING: This will move the robot!");
  RCLCPP_INFO(rclcpp::get_logger("move_homing"), "Make sure to have the emergency stop button at hand!");
  RCLCPP_INFO(rclcpp::get_logger("move_homing"), "Usage: ./move_homing [cable] (use 'cable' for cable position)");
  
  auto node = std::make_shared<MoveHomingNode>(use_cable_position);
  rclcpp::spin(node);
  
  RCLCPP_INFO(rclcpp::get_logger("move_homing"), "Homing procedure completed");
  rclcpp::shutdown();
  return 0;
}