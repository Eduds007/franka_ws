#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <memory>
#include <algorithm>

struct TactileData {
    cv::Mat depth_map;
    std::vector<cv::Point2f> marker_positions;
    cv::Mat contact_mask;
    double max_depth;
    double contact_area;
    cv::Vec3f estimated_force;
    cv::Vec3f estimated_torque;
    rclcpp::Time timestamp;
    bool is_valid;
};

class TactileSensorNode : public rclcpp::Node
{
public:
    TactileSensorNode() : Node("tactile_sensor_node")
    {
        // Declare parameters
        this->declare_parameter("frame_rate", 30.0);
        this->declare_parameter("force_scaling_factor", 100.0);  // N per mm of depth
        this->declare_parameter("gel_stiffness", 1000.0);        // N/m (estimated gel stiffness)
        this->declare_parameter("mm_per_pixel", 0.0634);         // GelSight Mini resolution
        this->declare_parameter("enable_debug_images", false);
        this->declare_parameter("contact_threshold", 30);        // Pixel intensity threshold
        this->declare_parameter("gaussian_blur_kernel", 5);
        
        // Get parameters
        force_scaling_factor_ = this->get_parameter("force_scaling_factor").as_double();
        gel_stiffness_ = this->get_parameter("gel_stiffness").as_double();
        mm_per_pixel_ = this->get_parameter("mm_per_pixel").as_double();
        enable_debug_ = this->get_parameter("enable_debug_images").as_bool();
        contact_threshold_ = this->get_parameter("contact_threshold").as_int();
        gaussian_kernel_ = this->get_parameter("gaussian_blur_kernel").as_int();
        
        // Initialize reference images (for depth calculation)
        left_ref_image_set_ = false;
        right_ref_image_set_ = false;
        
        // Subscribers
        left_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/gelsight/left/image_raw", 1,
            std::bind(&TactileSensorNode::left_image_callback, this, std::placeholders::_1));
            
        right_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/gelsight/right/image_raw", 1,
            std::bind(&TactileSensorNode::right_image_callback, this, std::placeholders::_1));
        
        // Publishers
        left_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/gelsight/left/force_estimation", 1);
        right_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/gelsight/right/force_estimation", 1);
            
        combined_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/gelsight/combined_force_estimation", 1);
            
        tactile_data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gelsight/tactile_analysis", 1);
        
        if (enable_debug_) {
            debug_left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/gelsight/left/debug_image", 1);
            debug_right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/gelsight/right/debug_image", 1);
        }
        
        RCLCPP_INFO(this->get_logger(), "Tactile sensor node initialized");
        RCLCPP_INFO(this->get_logger(), "Force scaling factor: %.2f N/mm", force_scaling_factor_);
        RCLCPP_INFO(this->get_logger(), "Resolution: %.4f mm/pixel", mm_per_pixel_);
        RCLCPP_INFO(this->get_logger(), "Waiting for reference images...");
    }

private:
    void left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat current_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            
            if (!left_ref_image_set_) {
                left_reference_image_ = current_image.clone();
                left_ref_image_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Left reference image captured");
                return;
            }
            
            TactileData tactile_data = analyze_tactile_image(current_image, left_reference_image_, msg->header.stamp);
            
            if (tactile_data.is_valid) {
                publish_force_estimation(tactile_data, left_force_pub_, "gelsight_left_optical_frame");
                
                if (enable_debug_) {
                    publish_debug_image(tactile_data, debug_left_pub_, msg->header);
                }
                
                left_tactile_data_ = tactile_data;
                update_combined_force_estimation();
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception (left): %s", e.what());
        }
    }
    
    void right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat current_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            
            if (!right_ref_image_set_) {
                right_reference_image_ = current_image.clone();
                right_ref_image_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Right reference image captured");
                return;
            }
            
            TactileData tactile_data = analyze_tactile_image(current_image, right_reference_image_, msg->header.stamp);
            
            if (tactile_data.is_valid) {
                publish_force_estimation(tactile_data, right_force_pub_, "gelsight_right_optical_frame");
                
                if (enable_debug_) {
                    publish_debug_image(tactile_data, debug_right_pub_, msg->header);
                }
                
                right_tactile_data_ = tactile_data;
                update_combined_force_estimation();
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception (right): %s", e.what());
        }
    }
    
    TactileData analyze_tactile_image(const cv::Mat& current_image, const cv::Mat& reference_image, const rclcpp::Time& timestamp)
    {
        TactileData result;
        result.timestamp = timestamp;
        result.is_valid = false;
        
        if (current_image.empty() || reference_image.empty()) {
            return result;
        }
        
        // 1. Convert to grayscale
        cv::Mat current_gray, ref_gray;
        cv::cvtColor(current_image, current_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(reference_image, ref_gray, cv::COLOR_BGR2GRAY);
        
        // 2. Calculate depth map (difference from reference)
        cv::Mat depth_diff;
        cv::absdiff(current_gray, ref_gray, depth_diff);
        
        // Apply Gaussian blur to reduce noise
        cv::GaussianBlur(depth_diff, result.depth_map, cv::Size(gaussian_kernel_, gaussian_kernel_), 0);
        
        // 3. Create contact mask based on threshold
        cv::threshold(result.depth_map, result.contact_mask, contact_threshold_, 255, cv::THRESH_BINARY);
        
        // 4. Calculate contact area (in mm²)
        int contact_pixels = cv::countNonZero(result.contact_mask);
        result.contact_area = contact_pixels * mm_per_pixel_ * mm_per_pixel_;
        
        if (contact_pixels < 10) {  // Minimum contact area
            return result;  // No significant contact
        }
        
        // 5. Calculate maximum depth in contact area
        double min_val, max_val;
        cv::minMaxLoc(result.depth_map, &min_val, &max_val, nullptr, nullptr, result.contact_mask);
        result.max_depth = max_val * mm_per_pixel_;  // Convert to mm
        
        // 6. Calculate center of pressure
        cv::Moments moments = cv::moments(result.contact_mask);
        if (moments.m00 > 0) {
            double cx = moments.m10 / moments.m00;
            double cy = moments.m01 / moments.m00;
            
            // 7. Estimate normal force based on depth and contact area
            // F = k * depth * area (simplified model)
            double normal_force = gel_stiffness_ * result.max_depth * 0.001 * result.contact_area * 0.000001; // Convert to N
            
            // Apply scaling factor
            normal_force *= force_scaling_factor_;
            
            result.estimated_force[0] = 0.0;  // Fx (lateral forces would need shear analysis)
            result.estimated_force[1] = 0.0;  // Fy
            result.estimated_force[2] = normal_force;  // Fz (normal force)
            
            // 8. Estimate torque based on center of pressure offset from center
            double image_center_x = current_image.cols / 2.0;
            double image_center_y = current_image.rows / 2.0;
            double offset_x = (cx - image_center_x) * mm_per_pixel_ * 0.001;  // Convert to m
            double offset_y = (cy - image_center_y) * mm_per_pixel_ * 0.001;
            
            result.estimated_torque[0] = normal_force * offset_y;  // Tx
            result.estimated_torque[1] = -normal_force * offset_x; // Ty
            result.estimated_torque[2] = 0.0;  // Tz (would need advanced analysis)
            
            result.is_valid = true;
        }
        
        return result;
    }
    
    void publish_force_estimation(const TactileData& tactile_data, 
                                 rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher,
                                 const std::string& frame_id)
    {
        auto wrench_msg = geometry_msgs::msg::WrenchStamped();
        wrench_msg.header.stamp = tactile_data.timestamp;
        wrench_msg.header.frame_id = frame_id;
        
        // Force
        wrench_msg.wrench.force.x = tactile_data.estimated_force[0];
        wrench_msg.wrench.force.y = tactile_data.estimated_force[1];
        wrench_msg.wrench.force.z = tactile_data.estimated_force[2];
        
        // Torque
        wrench_msg.wrench.torque.x = tactile_data.estimated_torque[0];
        wrench_msg.wrench.torque.y = tactile_data.estimated_torque[1];
        wrench_msg.wrench.torque.z = tactile_data.estimated_torque[2];
        
        publisher->publish(wrench_msg);
    }
    
    void update_combined_force_estimation()
    {
        if (!left_tactile_data_.is_valid || !right_tactile_data_.is_valid) {
            return;
        }
        
        auto combined_wrench = geometry_msgs::msg::WrenchStamped();
        combined_wrench.header.stamp = this->now();
        combined_wrench.header.frame_id = "gelsight_combined_frame";
        
        // Simple combination: sum of forces and torques
        combined_wrench.wrench.force.x = left_tactile_data_.estimated_force[0] + right_tactile_data_.estimated_force[0];
        combined_wrench.wrench.force.y = left_tactile_data_.estimated_force[1] + right_tactile_data_.estimated_force[1];
        combined_wrench.wrench.force.z = left_tactile_data_.estimated_force[2] + right_tactile_data_.estimated_force[2];
        
        combined_wrench.wrench.torque.x = left_tactile_data_.estimated_torque[0] + right_tactile_data_.estimated_torque[0];
        combined_wrench.wrench.torque.y = left_tactile_data_.estimated_torque[1] + right_tactile_data_.estimated_torque[1];
        combined_wrench.wrench.torque.z = left_tactile_data_.estimated_torque[2] + right_tactile_data_.estimated_torque[2];
        
        combined_force_pub_->publish(combined_wrench);
        
        // Publish detailed tactile analysis data
        auto tactile_array = std_msgs::msg::Float64MultiArray();
        tactile_array.data = {
            left_tactile_data_.max_depth,
            left_tactile_data_.contact_area,
            left_tactile_data_.estimated_force[2],
            right_tactile_data_.max_depth,
            right_tactile_data_.contact_area,
            right_tactile_data_.estimated_force[2],
            combined_wrench.wrench.force.x,
            combined_wrench.wrench.force.y,
            combined_wrench.wrench.force.z
        };
        
        tactile_data_pub_->publish(tactile_array);
        
        // Log periodically
        static int log_counter = 0;
        if (++log_counter % 30 == 0) {  // Log every 30 updates (1 second at 30Hz)
            RCLCPP_INFO(this->get_logger(), 
                       "Combined Force: [%.3f, %.3f, %.3f] N, Left: %.3f N, Right: %.3f N",
                       combined_wrench.wrench.force.x,
                       combined_wrench.wrench.force.y, 
                       combined_wrench.wrench.force.z,
                       left_tactile_data_.estimated_force[2],
                       right_tactile_data_.estimated_force[2]);
        }
    }
    
    void publish_debug_image(const TactileData& tactile_data, 
                            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
                            const std_msgs::msg::Header& header)
    {
        // Create debug visualization
        cv::Mat debug_image;
        cv::cvtColor(tactile_data.depth_map, debug_image, cv::COLOR_GRAY2BGR);
        
        // Apply colormap to depth
        cv::Mat depth_colored;
        cv::applyColorMap(tactile_data.depth_map, depth_colored, cv::COLORMAP_JET);
        
        // Overlay contact mask
        cv::Mat contact_colored;
        cv::cvtColor(tactile_data.contact_mask, contact_colored, cv::COLOR_GRAY2BGR);
        cv::addWeighted(depth_colored, 0.7, contact_colored, 0.3, 0, debug_image);
        
        // Add text overlay with force information
        std::string force_text = "Fz: " + std::to_string(tactile_data.estimated_force[2]) + " N";
        std::string depth_text = "Max depth: " + std::to_string(tactile_data.max_depth) + " mm";
        std::string area_text = "Area: " + std::to_string(tactile_data.contact_area) + " mm²";
        
        cv::putText(debug_image, force_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(debug_image, depth_text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(debug_image, area_text, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        // Convert back to ROS message and publish
        auto debug_msg = cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();
        publisher->publish(*debug_msg);
    }
    
    // Parameters
    double force_scaling_factor_;
    double gel_stiffness_;
    double mm_per_pixel_;
    bool enable_debug_;
    int contact_threshold_;
    int gaussian_kernel_;
    
    // Reference images for depth calculation
    cv::Mat left_reference_image_;
    cv::Mat right_reference_image_;
    bool left_ref_image_set_;
    bool right_ref_image_set_;
    
    // Current tactile data
    TactileData left_tactile_data_;
    TactileData right_tactile_data_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr left_force_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr right_force_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr combined_force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tactile_data_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_right_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TactileSensorNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("tactile_sensor_node"), 
                     "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}