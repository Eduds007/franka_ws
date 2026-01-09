#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class GelSightCameraNode : public rclcpp::Node
{
public:
    GelSightCameraNode() : Node("gelsight_camera_node")
    {
        // Declare parameters
        this->declare_parameter("camera_id", 0);
        this->declare_parameter("frame_rate", 30.0);
        this->declare_parameter("camera_side", "left");  // "left" or "right"
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        
        // Get parameters
        camera_id_ = this->get_parameter("camera_id").as_int();
        frame_rate_ = this->get_parameter("frame_rate").as_double();
        std::string camera_side = this->get_parameter("camera_side").as_string();
        int image_width = this->get_parameter("image_width").as_int();
        int image_height = this->get_parameter("image_height").as_int();
        
        // Set frame ID and topic names based on camera side
        frame_id_ = "gelsight_" + camera_side + "_optical_frame";
        image_topic_ = "/gelsight/" + camera_side + "/image_raw";
        camera_info_topic_ = "/gelsight/" + camera_side + "/camera_info";
        
        // Publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 1);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 1);
        
        // Initialize camera
        cap_ = std::make_unique<cv::VideoCapture>(camera_id_);
        
        if (!cap_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera with ID: %d", camera_id_);
            return;
        }
        
        // Set camera properties
        cap_->set(cv::CAP_PROP_FRAME_WIDTH, image_width);
        cap_->set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
        cap_->set(cv::CAP_PROP_FPS, frame_rate_);
        
        RCLCPP_INFO(this->get_logger(), "GelSight %s camera initialized successfully", camera_side.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Frame rate: %.1f Hz", frame_rate_);
        
        // Create timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / frame_rate_)),
            std::bind(&GelSightCameraNode::publish_image, this));
    }
    
    ~GelSightCameraNode()
    {
        if (cap_ && cap_->isOpened()) {
            cap_->release();
        }
    }

private:
    void publish_image()
    {
        cv::Mat frame;
        
        if (!cap_ || !cap_->read(frame)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Failed to capture frame from camera");
            return;
        }
        
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Captured empty frame");
            return;
        }
        
        // Convert OpenCV image to ROS message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        
        // Set timestamp and frame ID
        msg->header.stamp = this->now();
        msg->header.frame_id = frame_id_;
        
        // Publish image
        image_pub_->publish(*msg);
        
        // Create and publish basic camera info
        sensor_msgs::msg::CameraInfo camera_info_msg;
        camera_info_msg.header = msg->header;
        camera_info_msg.width = msg->width;
        camera_info_msg.height = msg->height;
        // Basic camera matrix (identity for now)
        camera_info_msg.k[0] = 500.0; // fx
        camera_info_msg.k[4] = 500.0; // fy
        camera_info_msg.k[2] = msg->width / 2.0;  // cx
        camera_info_msg.k[5] = msg->height / 2.0; // cy
        camera_info_msg.k[8] = 1.0;
        camera_info_pub_->publish(camera_info_msg);
        
        // Log publishing info periodically
        static int count = 0;
        if (++count % static_cast<int>(frame_rate_) == 0) {
            RCLCPP_INFO(this->get_logger(), "Published %d frames on %s", count, image_topic_.c_str());
        }
    }
    
    // Parameters
    int camera_id_;
    double frame_rate_;
    std::string frame_id_;
    std::string image_topic_;
    std::string camera_info_topic_;
    
    // Camera
    std::unique_ptr<cv::VideoCapture> cap_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GelSightCameraNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("gelsight_camera_node"), 
                     "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}