#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <Eigen/Dense>
#include <chrono>

class AdmittanceController : public rclcpp::Node
{
public:
    AdmittanceController() : Node("admittance_controller")
    {
        // Declare parameters
        this->declare_parameter("control_frequency", 125.0);  // Hz - matching Franka's control frequency
        this->declare_parameter("desired_force_z", 5.0);      // N - desired normal force (tension)
        this->declare_parameter("force_tolerance", 0.5);      // N - force tolerance
        
        // Admittance parameters (M, B, K matrices)
        this->declare_parameter("mass_x", 1.0);
        this->declare_parameter("mass_y", 1.0);
        this->declare_parameter("mass_z", 1.0);
        this->declare_parameter("damping_x", 50.0);
        this->declare_parameter("damping_y", 50.0);
        this->declare_parameter("damping_z", 50.0);
        this->declare_parameter("stiffness_x", 100.0);
        this->declare_parameter("stiffness_y", 100.0);
        this->declare_parameter("stiffness_z", 100.0);
        
        // Limits and safety
        this->declare_parameter("max_linear_velocity", 0.1);   // m/s
        this->declare_parameter("max_angular_velocity", 0.2);  // rad/s
        this->declare_parameter("max_linear_acceleration", 0.5); // m/s²
        
        // Control mode
        this->declare_parameter("control_mode", "position_based"); // "position_based" or "velocity_based"
        this->declare_parameter("enable_trajectory_following", true);
        
        // Get parameters
        control_freq_ = this->get_parameter("control_frequency").as_double();
        desired_force_z_ = this->get_parameter("desired_force_z").as_double();
        force_tolerance_ = this->get_parameter("force_tolerance").as_double();
        
        max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
        max_linear_accel_ = this->get_parameter("max_linear_acceleration").as_double();
        
        control_mode_ = this->get_parameter("control_mode").as_string();
        enable_trajectory_ = this->get_parameter("enable_trajectory_following").as_bool();
        
        // Initialize admittance matrices
        M_ = Eigen::Matrix3d::Zero();
        B_ = Eigen::Matrix3d::Zero();
        K_ = Eigen::Matrix3d::Zero();
        
        M_(0,0) = this->get_parameter("mass_x").as_double();
        M_(1,1) = this->get_parameter("mass_y").as_double();
        M_(2,2) = this->get_parameter("mass_z").as_double();
        
        B_(0,0) = this->get_parameter("damping_x").as_double();
        B_(1,1) = this->get_parameter("damping_y").as_double();
        B_(2,2) = this->get_parameter("damping_z").as_double();
        
        K_(0,0) = this->get_parameter("stiffness_x").as_double();
        K_(1,1) = this->get_parameter("stiffness_y").as_double();
        K_(2,2) = this->get_parameter("stiffness_z").as_double();
        
        // Initialize state variables
        position_ = Eigen::Vector3d::Zero();
        velocity_ = Eigen::Vector3d::Zero();
        acceleration_ = Eigen::Vector3d::Zero();
        desired_position_ = Eigen::Vector3d::Zero();
        
        current_force_ = Eigen::Vector3d::Zero();
        desired_force_ = Eigen::Vector3d(0.0, 0.0, desired_force_z_);
        
        dt_ = 1.0 / control_freq_;
        
        // Control state
        is_active_ = false;
        force_data_received_ = false;
        
        // Subscribers
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/gelsight/combined_force_estimation", 1,
            std::bind(&AdmittanceController::force_callback, this, std::placeholders::_1));
            
        trajectory_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/admittance_controller/desired_pose", 1,
            std::bind(&AdmittanceController::trajectory_callback, this, std::placeholders::_1));
        
        // Publishers
        if (control_mode_ == "velocity_based") {
            velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/admittance_controller/cmd_vel", 1);
        } else {
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/admittance_controller/cmd_pose", 1);
        }
        
        // Status and debug publishers
        status_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/admittance_controller/status", 1);
        debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/admittance_controller/debug", 1);
        
        // Services
        activate_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/admittance_controller/activate",
            std::bind(&AdmittanceController::activate_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));
                     
        deactivate_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/admittance_controller/deactivate",
            std::bind(&AdmittanceController::deactivate_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Control timer
        control_timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(1000000.0 / control_freq_)),
            std::bind(&AdmittanceController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Admittance Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Control frequency: %.1f Hz", control_freq_);
        RCLCPP_INFO(this->get_logger(), "Desired force: [%.2f, %.2f, %.2f] N", 
                   desired_force_[0], desired_force_[1], desired_force_[2]);
        RCLCPP_INFO(this->get_logger(), "Control mode: %s", control_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "Controller inactive. Use service to activate.");
    }

private:
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        // Update current force from GelSight tactile sensor
        current_force_[0] = msg->wrench.force.x;
        current_force_[1] = msg->wrench.force.y;
        current_force_[2] = msg->wrench.force.z;
        
        force_data_received_ = true;
        last_force_time_ = this->now();
        
        // Log force data periodically
        static int force_log_counter = 0;
        if (++force_log_counter % static_cast<int>(control_freq_) == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "Measured force: [%.3f, %.3f, %.3f] N", 
                       current_force_[0], current_force_[1], current_force_[2]);
        }
    }
    
    void trajectory_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (enable_trajectory_) {
            // Update desired position from trajectory
            desired_position_[0] = msg->pose.position.x;
            desired_position_[1] = msg->pose.position.y;
            desired_position_[2] = msg->pose.position.z;
            
            RCLCPP_DEBUG(this->get_logger(), 
                        "Updated desired position: [%.3f, %.3f, %.3f]",
                        desired_position_[0], desired_position_[1], desired_position_[2]);
        }
    }
    
    void control_loop()
    {
        if (!is_active_ || !force_data_received_) {
            return;
        }
        
        // Check if force data is fresh (timeout protection)
        auto now = this->now();
        if ((now - last_force_time_).seconds() > 0.1) {  // 100ms timeout
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Force data timeout - stopping controller");
            return;
        }
        
        // Calculate force error
        Eigen::Vector3d force_error = current_force_ - desired_force_;
        
        // Admittance control law: M*x_ddot + B*x_dot + K*(x - x_d) = F_ext - F_des
        // Rearranged: x_ddot = M^-1 * (F_error - B*x_dot - K*(x - x_d))
        
        Eigen::Vector3d position_error = position_ - desired_position_;
        
        // Calculate acceleration using admittance equation
        Eigen::Vector3d new_acceleration = M_.inverse() * (force_error - B_ * velocity_ - K_ * position_error);
        
        // Apply acceleration limits
        for (int i = 0; i < 3; ++i) {
            if (std::abs(new_acceleration[i]) > max_linear_accel_) {
                new_acceleration[i] = std::copysign(max_linear_accel_, new_acceleration[i]);
            }
        }
        
        // Integrate to get velocity (Euler integration)
        velocity_ += new_acceleration * dt_;
        
        // Apply velocity limits
        double vel_norm = velocity_.norm();
        if (vel_norm > max_linear_vel_) {
            velocity_ = velocity_ * (max_linear_vel_ / vel_norm);
        }
        
        // Integrate to get position
        position_ += velocity_ * dt_;
        acceleration_ = new_acceleration;
        
        // Publish control commands
        if (control_mode_ == "velocity_based") {
            publish_velocity_command();
        } else {
            publish_position_command();
        }
        
        // Publish status and debug information
        publish_status();
        publish_debug_info();
    }
    
    void publish_velocity_command()
    {
        auto cmd_vel = geometry_msgs::msg::TwistStamped();
        cmd_vel.header.stamp = this->now();
        cmd_vel.header.frame_id = "base_link";
        
        cmd_vel.twist.linear.x = velocity_[0];
        cmd_vel.twist.linear.y = velocity_[1];
        cmd_vel.twist.linear.z = velocity_[2];
        
        // Angular velocity can be added here if needed for orientation control
        cmd_vel.twist.angular.x = 0.0;
        cmd_vel.twist.angular.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        
        velocity_pub_->publish(cmd_vel);
    }
    
    void publish_position_command()
    {
        auto cmd_pose = geometry_msgs::msg::PoseStamped();
        cmd_pose.header.stamp = this->now();
        cmd_pose.header.frame_id = "base_link";
        
        cmd_pose.pose.position.x = position_[0];
        cmd_pose.pose.position.y = position_[1];
        cmd_pose.pose.position.z = position_[2];
        
        // Keep current orientation (identity quaternion)
        cmd_pose.pose.orientation.w = 1.0;
        cmd_pose.pose.orientation.x = 0.0;
        cmd_pose.pose.orientation.y = 0.0;
        cmd_pose.pose.orientation.z = 0.0;
        
        pose_pub_->publish(cmd_pose);
    }
    
    void publish_status()
    {
        auto status = std_msgs::msg::Float64MultiArray();
        status.data = {
            static_cast<double>(is_active_),
            current_force_[0], current_force_[1], current_force_[2],
            desired_force_[0], desired_force_[1], desired_force_[2],
            std::abs(current_force_[2] - desired_force_[2]),  // force error magnitude
            velocity_.norm(),
            position_.norm()
        };
        
        status_pub_->publish(status);
    }
    
    void publish_debug_info()
    {
        auto debug = std_msgs::msg::Float64MultiArray();
        debug.data = {
            position_[0], position_[1], position_[2],
            velocity_[0], velocity_[1], velocity_[2],
            acceleration_[0], acceleration_[1], acceleration_[2],
            desired_position_[0], desired_position_[1], desired_position_[2]
        };
        
        debug_pub_->publish(debug);
    }
    
    void activate_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!force_data_received_) {
            response->success = false;
            response->message = "No force data received. Make sure tactile_sensor_node is running.";
            RCLCPP_WARN(this->get_logger(), "Activation failed: %s", response->message.c_str());
            return;
        }
        
        // Reset state
        velocity_ = Eigen::Vector3d::Zero();
        acceleration_ = Eigen::Vector3d::Zero();
        
        is_active_ = true;
        response->success = true;
        response->message = "Admittance controller activated";
        
        RCLCPP_INFO(this->get_logger(), "Admittance controller activated");
    }
    
    void deactivate_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        is_active_ = false;
        
        // Reset velocities for safety
        velocity_ = Eigen::Vector3d::Zero();
        acceleration_ = Eigen::Vector3d::Zero();
        
        response->success = true;
        response->message = "Admittance controller deactivated";
        
        RCLCPP_INFO(this->get_logger(), "Admittance controller deactivated");
    }
    
    // Parameters
    double control_freq_;
    double desired_force_z_;
    double force_tolerance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double max_linear_accel_;
    std::string control_mode_;
    bool enable_trajectory_;
    double dt_;
    
    // Admittance matrices (M, B, K)
    Eigen::Matrix3d M_, B_, K_;
    
    // State variables
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d acceleration_;
    Eigen::Vector3d desired_position_;
    Eigen::Vector3d current_force_;
    Eigen::Vector3d desired_force_;
    
    // Control state
    bool is_active_;
    bool force_data_received_;
    rclcpp::Time last_force_time_;
    
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trajectory_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr activate_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr deactivate_service_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AdmittanceController>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("admittance_controller"), 
                     "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}