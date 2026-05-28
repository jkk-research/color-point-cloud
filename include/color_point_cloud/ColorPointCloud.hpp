//
// Created by bzeren on 05.10.2023.
//

#pragma once

#include "color_point_cloud/data_type/Camera.hpp"
#include "color_point_cloud/data_type/PointCloudType.hpp"
#include "color_point_cloud/utils/TransformeProvider.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "cv_bridge/cv_bridge.h"

#include <message_filters/simple_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>

#include <array>

template<typename MsgT>
class AdjustableStampFilter : public message_filters::SimpleFilter<MsgT> {
public:
    void add(const typename MsgT::ConstSharedPtr &msg, int64_t offset_ns = 0) {
        auto adjusted_msg = std::make_shared<MsgT>(*msg);
        if (offset_ns != 0) {
            const auto adjusted_time = rclcpp::Time(adjusted_msg->header.stamp) + rclcpp::Duration::from_nanoseconds(offset_ns);
            adjusted_msg->header.stamp = adjusted_time;
        }
        this->signalMessage(adjusted_msg);
    }
};

namespace color_point_cloud {
    class ColorPointCloud : public rclcpp::Node {
    public:
        explicit ColorPointCloud(const rclcpp::NodeOptions &options);

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

        double timeout_sec_;
        bool use_reliable_qos_;
        bool debug_;
        bool debug_camera_overlap_red_;
        bool use_approximate_sync_;
        bool approximate_sync_active_;
        double approximate_sync_slop_ms_;
        int approximate_sync_queue_size_;

        std::string point_cloud_topic_;

        std::string point_cloud_frame_id_;

        ImageType image_type_;

        std::string image_topic_last_name_;

        std::string camera_info_topic_last_name_;

        std::vector<std::string> camera_topics_;

        bool use_compressed_image_;
        std::vector<double> camera_time_offsets_ms_;

        std::map<std::string, CameraTypePtr> camera_type_stdmap_;
        std::map<std::string, bool> missing_transform_warned_;
        std::map<std::string, std::string> camera_state_log_;
        std::map<std::string, int64_t> camera_time_offsets_ns_;
        std::map<std::string, size_t> camera_sync_indices_;

        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;
        std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> compressed_image_subscribers_;
        std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subscribers_;

        using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2,
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image>;

        AdjustableStampFilter<sensor_msgs::msg::PointCloud2> point_cloud_sync_filter_;
        std::array<AdjustableStampFilter<sensor_msgs::msg::Image>, 2> camera_image_sync_filters_;
        std::unique_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> approximate_synchronizer_;

        rclcpp::ReliabilityPolicy point_cloud_subscription_reliability_;
        size_t processed_point_cloud_frames_;
                                                                                          
        void timer_callback();

        void point_cloud_input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

        void point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

        void approximate_sync_callback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &point_cloud_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr &camera0_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr &camera1_msg);

        void initialize_approximate_sync();

        void create_or_update_point_cloud_subscription();

        rclcpp::ReliabilityPolicy detect_point_cloud_subscription_reliability();

        void log_camera_state(const std::string &camera_topic, const std::string &state);

        static const char *reliability_to_string(rclcpp::ReliabilityPolicy reliability_policy);

        TransformProviderConstPtr transform_provider_ptr_;
    };
} // namespace color_point_cloud
