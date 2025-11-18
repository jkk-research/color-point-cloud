//
// Created by bzeren on 05.10.2023.
//

#include "color_point_cloud/ColorPointCloud.hpp"

namespace color_point_cloud {
    ColorPointCloud::ColorPointCloud(const rclcpp::NodeOptions &options) : Node("color_point_cloud_node", options),
                                                                           transform_provider_ptr_(
                                                                                   std::make_shared<TransformProvider>(
                                                                                           this->get_clock())) {
        RCLCPP_INFO(this->get_logger(), "ColorPointCloud node started");

        // Declare and get parameters
        {
            // timeout_sec
            this->declare_parameter<double>("timeout_sec", 0.1);
            timeout_sec_ = this->get_parameter("timeout_sec").as_double();

            this->declare_parameter<std::string>("point_cloud_topic", "/points_raw");
            point_cloud_topic_ = this->get_parameter("point_cloud_topic").as_string();

            this->declare_parameter<std::string>("point_cloud_frame_id", "lidar");
            point_cloud_frame_id_ = this->get_parameter("point_cloud_frame_id").as_string();

            this->declare_parameter<int>("image_type", 0);
            image_type_ = static_cast<ImageType>(this->get_parameter("image_type").as_int());

            this->declare_parameter<std::string>("image_topic_last_name", "/image_raw");
            image_topic_last_name_ = this->get_parameter("image_topic_last_name").as_string();

            this->declare_parameter<std::string>("camera_info_topic_last_name", "/camera_info");
            camera_info_topic_last_name_ = this->get_parameter("camera_info_topic_last_name").as_string();

            this->declare_parameter<bool>("use_compressed_image", false);
            use_compressed_image_ = this->get_parameter("use_compressed_image").as_bool();

            // camera_topics
            this->declare_parameter<std::vector<std::string>>("camera_topics", std::vector<std::string>());
            camera_topics_ = this->get_parameter("camera_topics").as_string_array();

            for (auto &camera_topic: camera_topics_) {
                RCLCPP_INFO(this->get_logger(), "camera_topic: %s", camera_topic.c_str());
            }
        }

        // Create camera object, create subscriber to image and camera_info
        {
            for (const auto &camera_topic: camera_topics_) {
                std::string image_topic = camera_topic + image_topic_last_name_;
                std::string camera_info_topic = camera_topic + camera_info_topic_last_name_;

                CameraTypePtr camera_type_ptr = std::make_shared<CameraType>(image_topic, camera_info_topic);
                camera_type_stdmap_[camera_topic] = camera_type_ptr;

                if (use_compressed_image_) {
                    this->compressed_image_subscribers_.push_back(
                            this->create_subscription<sensor_msgs::msg::CompressedImage>(
                                    image_topic, rclcpp::SensorDataQoS(),
                                    [this, image_topic, camera_topic](
                                            const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) {
                                        // RCLCPP_INFO(this->get_logger(), "Received compressed image on topic %s", camera_topic.c_str());
                                        try {
                                            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
                                            if (image.empty()) {
                                                RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image on topic %s", camera_topic.c_str());
                                                return;
                                            }
                                            cv_bridge::CvImage cv_image;
                                            cv_image.header = msg->header;
                                            cv_image.encoding = sensor_msgs::image_encodings::BGR8;
                                            cv_image.image = image;
                                            auto image_msg = cv_image.toImageMsg();
                                            camera_type_stdmap_[camera_topic]->set_image_msg(image_msg);
                                        } catch (cv::Exception &e) {
                                            RCLCPP_ERROR(this->get_logger(), "OpenCV exception on topic %s: %s", camera_topic.c_str(), e.what());
                                        }
                                    }));
                } else {
                    this->image_subscribers_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
                            image_topic, rclcpp::SensorDataQoS(),
                            [this, image_topic, camera_topic](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                                // RCLCPP_INFO(this->get_logger(), "Received image on topic %s", camera_topic.c_str());
                                camera_type_stdmap_[camera_topic]->set_image_msg(msg);
                            }));
                }

                this->camera_info_subscribers_.push_back(this->create_subscription<sensor_msgs::msg::CameraInfo>(
                        camera_info_topic, rclcpp::SensorDataQoS(), [this, camera_info_topic, camera_topic](
                                const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
                            // RCLCPP_INFO(this->get_logger(), "Received camera_info on topic %s", camera_topic.c_str());
                            camera_type_stdmap_[camera_topic]->set_camera_info(msg);
                        }));
            }
        }

        {
            point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    point_cloud_topic_, rclcpp::SensorDataQoS(),
                    std::bind(&ColorPointCloud::point_cloud_callback, this, std::placeholders::_1));

            point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/points_color", rclcpp::SensorDataQoS());
        }

        // Set timer
        {
            const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(timeout_sec_));
            timer_ = this->create_wall_timer(period_ns,
                                             std::bind(&ColorPointCloud::timer_callback, this));
        }

    }

    void ColorPointCloud::timer_callback() {
        //RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
        for (const auto &pair: camera_type_stdmap_) {
            if (pair.second->get_image_msg() == nullptr || pair.second->get_camera_info() == nullptr) {
//                RCLCPP_INFO(this->get_logger(), "Image or camera info is null for topic: %s", pair.first.c_str());
                continue;
            }

            if (!pair.second->is_info_initialized()) {
                auto cam_info = pair.second->get_camera_info();
                RCLCPP_INFO(this->get_logger(), "Camera info is setting: %s", pair.first.c_str());
                RCLCPP_INFO(this->get_logger(), "Camera info - width: %d, height: %d, K[0]: %f, frame_id: %s", 
                    cam_info->width, cam_info->height, cam_info->k[0], cam_info->header.frame_id.c_str());
                pair.second->set_camera_utils(cam_info);
                if (!pair.second->is_info_initialized()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera utils for: %s", pair.first.c_str());
                }
            }

            if (!pair.second->is_transform_initialized() && pair.second->is_info_initialized()) {
                std::optional<geometry_msgs::msg::TransformStamped> transform = (*transform_provider_ptr_)(
                        pair.second->get_camera_frame_id(), point_cloud_frame_id_);
                if (!transform.has_value()) {
                    continue;
                }
                pair.second->set_lidar_to_camera_matrix(transform.value());
                pair.second->set_lidar_to_camera_projection_matrix();
            }
        }
    }

    void ColorPointCloud::point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {

        sensor_msgs::msg::PointCloud2 cloud_color_msg;
        std::for_each(camera_type_stdmap_.begin(), camera_type_stdmap_.end(),
                      [this, &cloud_color_msg, msg](std::pair<std::string, CameraTypePtr> pair) {
                          if (pair.second->get_image_msg() == nullptr) {
                              RCLCPP_WARN(this->get_logger(), "Camera %s: don't receive image", pair.first.c_str());
                              return;
                          }
                          if (pair.second->get_camera_info() == nullptr) {
                              RCLCPP_WARN(this->get_logger(), "Camera %s: don't receive camera_info", pair.first.c_str());
                              return;
                          }
                        //   if (!pair.second->is_info_initialized()) {
                        //       RCLCPP_WARN(this->get_logger(), "Camera %s: can't init camera utils", pair.first.c_str());
                        //       return;
                        //   }
                          if (!pair.second->is_transform_initialized()) {
                              RCLCPP_WARN(this->get_logger(), "Camera %s: can't get transform", pair.first.c_str());
                              return;
                          }
                          RCLCPP_INFO(this->get_logger(), "AAAAA");

                          pair.second->set_cv_image(pair.second->get_image_msg(), image_type_);

                          // Check if input has ring and intensity fields
                          bool has_ring = false;
                          bool has_intensity = false;
                          for (const auto& field : msg->fields) {
                              if (field.name == "ring") has_ring = true;
                              if (field.name == "intensity") has_intensity = true;
                          }

                          sensor_msgs::PointCloud2Modifier modifier(cloud_color_msg);
                          if (has_ring && has_intensity) {
                              modifier.setPointCloud2Fields(6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                                            sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                                            sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
                                                            sensor_msgs::msg::PointField::FLOAT32, "ring", 1,
                                                            sensor_msgs::msg::PointField::UINT16, "intensity", 1,
                                                            sensor_msgs::msg::PointField::FLOAT32);
                          } else {
                              modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                                            sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                                            sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
                                                            sensor_msgs::msg::PointField::FLOAT32);
                          }
                          modifier.resize(msg->width * msg->height);

                          sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_color_msg, "x");
                          sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_color_msg, "y");
                          sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_color_msg, "z");
                          sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_color_msg, "r");
                          sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_color_msg, "g");
                          sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_color_msg, "b");

                          sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*msg, "x");
                          sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*msg, "y");
                          sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*msg, "z");
                          
                          // Optional iterators for ring and intensity
                          std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint16_t>> iter_ring;
                          std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_intensity;
                          std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> iter_ring_in;
                          std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_intensity_in;
                          
                          if (has_ring && has_intensity) {
                              try {
                                  iter_ring = std::make_unique<sensor_msgs::PointCloud2Iterator<uint16_t>>(cloud_color_msg, "ring");
                                  iter_ring_in = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(*msg, "ring");
                              } catch (const std::exception& e) {
                                  RCLCPP_WARN(this->get_logger(), "Failed to create ring iterator: %s. Skipping ring field.", e.what());
                                  has_ring = false;
                              }
                              
                              try {
                                  iter_intensity = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(cloud_color_msg, "intensity");
                                  iter_intensity_in = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
                              } catch (const std::exception& e) {
                                  RCLCPP_WARN(this->get_logger(), "Failed to create intensity iterator: %s. Skipping intensity field.", e.what());
                                  has_intensity = false;
                              }
                          }
                          
                          for (size_t i = 0; i < msg->width * msg->height; ++i) {
                              float px = iter_x_in[0];
                              float py = iter_y_in[0];
                              float pz = iter_z_in[0];

                              Eigen::Vector4d point4d(px, py, pz, 1.0);

                              Eigen::Vector3d point3d_transformed_camera =
                                      pair.second->get_lidar_to_camera_projection_matrix() * point4d;

                              Eigen::Vector2d point2d_transformed_camera = Eigen::Vector2d(
                                      point3d_transformed_camera[0] / point3d_transformed_camera[2],
                                      point3d_transformed_camera[1] / point3d_transformed_camera[2]);

                              double x = point2d_transformed_camera[0];
                              double y = point2d_transformed_camera[1];


                              iter_x[0] = px;
                              iter_y[0] = py;
                              iter_z[0] = pz;
                              
                              if (has_ring && iter_ring_in) {
                                  (*iter_ring)[0] = (*iter_ring_in)[0];
                              }
                              if (has_intensity && iter_intensity_in) {
                                  (*iter_intensity)[0] = (*iter_intensity_in)[0];
                              }

                              if (x >= 0 && x < pair.second->get_image_width() && y >= 0 &&
                                  y < pair.second->get_image_height() &&
                                  point3d_transformed_camera[2] > 0) {

                                  cv::Vec3b color = pair.second->get_cv_image().at<cv::Vec3b>(cv::Point(x, y));

                                  if (pair.second->get_image_msg()->encoding == "rgb8") {
                                      iter_r[0] = color[0];
                                      iter_g[0] = color[1];
                                      iter_b[0] = color[2];
                                  } else if (pair.second->get_image_msg()->encoding == "bgr8") {
                                      iter_r[0] = color[2];
                                      iter_g[0] = color[1];
                                      iter_b[0] = color[0];
                                  } else {
                                      iter_r[0] = color[2];
                                      iter_g[0] = color[1];
                                      iter_b[0] = color[0];
                                  }
                              }

                              ++iter_x;
                              ++iter_y;
                              ++iter_z;
                              ++iter_r;
                              ++iter_g;
                              ++iter_b;
                              
                              ++iter_x_in;
                              ++iter_y_in;
                              ++iter_z_in;
                              
                              if (has_ring && iter_ring && iter_ring_in) {
                                  ++(*iter_ring);
                                  ++(*iter_ring_in);
                              }
                              if (has_intensity && iter_intensity && iter_intensity_in) {
                                  ++(*iter_intensity);
                                  ++(*iter_intensity_in);
                              }
                          }
//                          cv::imshow(pair.first, pair.second->get_cv_image());
//                          cv::waitKey(1);
                      });

        cloud_color_msg.header = msg->header;
        cloud_color_msg.height = 1;
        cloud_color_msg.width = msg->width * msg->height;
        point_cloud_publisher_->publish(cloud_color_msg);
    }


} // namespace color_point_cloud