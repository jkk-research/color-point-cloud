#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

class CuttedPointCloud : public rclcpp::Node
{
public:
    CuttedPointCloud() : Node("cutted_pointcloud_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/luminar_pcl");
        this->declare_parameter<std::string>("output_topic", "/luminar_pcl_cutted");
        this->declare_parameter<double>("cut_angle_right_deg", 57.5);  // Cut from right side (positive angles)
        this->declare_parameter<double>("cut_angle_left_deg", 57.5);   // Cut from left side (negative angles)
        this->declare_parameter<double>("min_distance_x", 3.0);  // Minimum distance in X (forward) direction in meters
        this->declare_parameter<bool>("use_reliable_qos", true);  // true for rosbag, false for live sensor
        
        // Get parameters
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        cut_angle_right_deg_ = this->get_parameter("cut_angle_right_deg").as_double();
        cut_angle_left_deg_ = this->get_parameter("cut_angle_left_deg").as_double();
        min_distance_x_ = this->get_parameter("min_distance_x").as_double();
        bool use_reliable = this->get_parameter("use_reliable_qos").as_bool();
        
        // Convert to radians
        // For forward-looking lidar (180° FOV): -90° (left) to +90° (right)
        // We cut from the edges, so we keep the center
        min_angle_rad_ = -M_PI / 2.0 + (cut_angle_left_deg_ * M_PI / 180.0);   // Left boundary
        max_angle_rad_ = M_PI / 2.0 - (cut_angle_right_deg_ * M_PI / 180.0);   // Right boundary
        
        double kept_angle = 180.0 - cut_angle_left_deg_ - cut_angle_right_deg_;
        
        RCLCPP_INFO(this->get_logger(), "PointCloud Cutter Node Started");
        RCLCPP_INFO(this->get_logger(), "Cutting right side: %.1f°, left side: %.1f°", 
                    cut_angle_right_deg_, cut_angle_left_deg_);
        RCLCPP_INFO(this->get_logger(), "Keeping center %.1f° (from %.1f° to %.1f°)", 
                    kept_angle, min_angle_rad_ * 180.0 / M_PI, max_angle_rad_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Min distance in X (forward): %.2f m (cutting 0-%.2f m)", 
                    min_distance_x_, min_distance_x_);
        
        // Configure QoS based on parameter
        auto qos = use_reliable ? 
            rclcpp::QoS(1).reliable().durability_volatile().keep_last(1) : 
            rclcpp::SensorDataQoS().keep_last(1);
        
        RCLCPP_INFO(this->get_logger(), "Using %s QoS", use_reliable ? "RELIABLE" : "BEST_EFFORT");
        
        // Create subscriber and publisher
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&CuttedPointCloud::pointcloud_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, qos);
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        frame_count_++;
        
        // Check if input has required fields
        bool has_ring = false;
        bool has_intensity = false;
        for (const auto& field : msg->fields) {
            if (field.name == "ring") has_ring = true;
            if (field.name == "intensity") has_intensity = true;
        }
        
        // Create output point cloud structure
        sensor_msgs::msg::PointCloud2 output_msg;
        sensor_msgs::PointCloud2Modifier modifier(output_msg);
        
        if (has_ring && has_intensity) {
            modifier.setPointCloud2Fields(5,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
                "ring", 1, sensor_msgs::msg::PointField::UINT16);
        } else if (has_intensity) {
            modifier.setPointCloud2Fields(4,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        } else if (has_ring) {
            modifier.setPointCloud2Fields(4,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "ring", 1, sensor_msgs::msg::PointField::UINT16);
        } else {
            modifier.setPointCloud2Fields(3,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        }
        
        // Pre-allocate with same size (will resize later)
        modifier.resize(msg->width * msg->height);
        
        // Create iterators for output
        sensor_msgs::PointCloud2Iterator<float> iter_x_out(output_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y_out(output_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z_out(output_msg, "z");
        
        std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint16_t>> iter_ring_out;
        std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_intensity_out;
        
        if (has_ring) {
            try {
                iter_ring_out = std::make_unique<sensor_msgs::PointCloud2Iterator<uint16_t>>(output_msg, "ring");
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to create ring output iterator: %s", e.what());
                has_ring = false;
            }
        }
        if (has_intensity) {
            try {
                iter_intensity_out = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(output_msg, "intensity");
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to create intensity output iterator: %s", e.what());
                has_intensity = false;
            }
        }
        
        // Create iterators for input
        sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*msg, "z");
        
        std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> iter_ring_in;
        std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_intensity_in;
        
        if (has_ring) {
            try {
                iter_ring_in = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(*msg, "ring");
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to create ring input iterator: %s", e.what());
                has_ring = false;
            }
        }
        if (has_intensity) {
            try {
                iter_intensity_in = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to create intensity input iterator: %s", e.what());
                has_intensity = false;
            }
        }
        
        size_t kept_points = 0;
        size_t total_points = msg->width * msg->height;
        
        // Process each point
        for (size_t i = 0; i < total_points; ++i) {
            float x = iter_x_in[0];
            float y = iter_y_in[0];
            float z = iter_z_in[0];
            
            // Calculate angle in XY plane (azimuth angle)
            // For forward-looking sensor: x is forward, y is left/right
            // atan2(y, x) gives angle from positive x-axis in range [-pi, pi]
            // Positive angles = right side, negative angles = left side
            double angle = std::atan2(y, x);
            
            // Keep points within the specified angle range and distance
            // Cut cut_angle_left from left (-90°) and cut_angle_right from right (+90°)
            // Also cut points closer than min_distance_x in forward direction
            if (angle >= min_angle_rad_ && angle <= max_angle_rad_ && x >= min_distance_x_) {
                iter_x_out[0] = x;
                iter_y_out[0] = y;
                iter_z_out[0] = z;
                
                if (has_ring && iter_ring_in && iter_ring_out) {
                    (*iter_ring_out)[0] = (*iter_ring_in)[0];
                }
                if (has_intensity && iter_intensity_in && iter_intensity_out) {
                    (*iter_intensity_out)[0] = (*iter_intensity_in)[0];
                }
                
                ++iter_x_out;
                ++iter_y_out;
                ++iter_z_out;
                
                if (has_ring && iter_ring_out) {
                    ++(*iter_ring_out);
                }
                if (has_intensity && iter_intensity_out) {
                    ++(*iter_intensity_out);
                }
                
                kept_points++;
            }
            
            // Advance input iterators
            ++iter_x_in;
            ++iter_y_in;
            ++iter_z_in;
            
            if (has_ring && iter_ring_in) {
                ++(*iter_ring_in);
            }
            if (has_intensity && iter_intensity_in) {
                ++(*iter_intensity_in);
            }
        }
        
        // Resize to actual number of kept points
        modifier.resize(kept_points);
        
        // Set header
        output_msg.header = msg->header;
        output_msg.height = 1;
        output_msg.width = static_cast<uint32_t>(kept_points);
        
        // Publish
        if (kept_points > 0) {
            publisher_->publish(output_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "All points filtered out, not publishing");
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Log statistics every 30 frames
        if (frame_count_ % 30 == 0) {
            double kept_percentage = (static_cast<double>(kept_points) / total_points) * 100.0;
            RCLCPP_INFO(this->get_logger(), 
                "Frame %zu: Kept %zu/%zu points (%.1f%%), processing took %ld ms", 
                frame_count_, kept_points, total_points, kept_percentage, duration.count());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    
    std::string input_topic_;
    std::string output_topic_;
    double cut_angle_right_deg_;
    double cut_angle_left_deg_;
    double min_angle_rad_;  // Minimum angle to keep (left boundary)
    double max_angle_rad_;  // Maximum angle to keep (right boundary)
    double min_distance_x_;  // Minimum distance in X (forward) direction - cut closer points
    
    size_t frame_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CuttedPointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
