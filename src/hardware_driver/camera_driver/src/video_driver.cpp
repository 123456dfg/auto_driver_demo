// std
#include <filesystem>
#include <string>

// ros2
#include <rclcpp/rclcpp.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

// 3rd party
#include <opencv2/opencv.hpp>

//utils
#include "utils/url_resolver.hpp"


namespace camera_driver
{
    class VideoDriverNode : public rclcpp::Node
    {
    private:
        image_transport::CameraPublisher camera_pub_;
        std::string video_path_;
        bool is_loop_;
        cv::VideoCapture cap_;
        cv::Mat frame_;
        sensor_msgs::msg::Image::SharedPtr image_msg_;
        sensor_msgs::msg::CameraInfo camera_info_;
        std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
        rclcpp::TimerBase::SharedPtr timer_;
        int start_frame_;
        int frame_cnt_;

    public:
        explicit VideoDriverNode(const rclcpp::NodeOptions &options) : Node("video_driver", options), frame_cnt_(0)
        {
            RCLCPP_INFO(this->get_logger(), "Starting VideoDriverNode");
            // params setting
            video_path_ = utils::URLResolver::getPackageFileName(this->declare_parameter("video_path", "package://resource/video/traffic_sign1.mp4"));
            RCLCPP_INFO(this->get_logger(), "%s", video_path_.c_str());
            std::string camera_info_url = this->declare_parameter("camera_info_url", "package://driver_bringup/config/camera_info.yaml");
            std::string frame_id = this->declare_parameter("frame_id", "camera_optical_frame");
            int max_fps = this->declare_parameter("max_fps", 60);
            start_frame_ = this->declare_parameter("start_frame", 0);
            is_loop_ = this->declare_parameter("keep_looping", true);

            std::filesystem::path video_file(video_path_);

            if (!std::filesystem::exists(video_file))
            {
                RCLCPP_ERROR(this->get_logger(), "Video file %s not exist!", video_file.string().c_str());
                rclcpp::shutdown();
                return;
            }
            cap_.open(video_path_);
            if (!cap_.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Video file open failed!");
                rclcpp::shutdown();
                return;
            }

            // Set image msg
            image_msg_ = std::make_shared<sensor_msgs::msg::Image>();
            image_msg_->header.frame_id = frame_id;
            image_msg_->encoding = sensor_msgs::image_encodings::BGR8;
            image_msg_->width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
            image_msg_->height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
            image_msg_->step = image_msg_->width * 3;
            image_msg_->data.resize(image_msg_->step * image_msg_->height);

            // Set camera info
            camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
                this, "camera_driver", "file://" + video_path_);

            if (camera_info_manager_->validateURL(camera_info_url))
            {
                camera_info_manager_->loadCameraInfo(camera_info_url);
                camera_info_ = camera_info_manager_->getCameraInfo();
            }
            else
            {
                camera_info_manager_->setCameraName(video_path_);
                sensor_msgs::msg::CameraInfo camera_info;
                camera_info.header.frame_id = "camera_optical_frame";
                camera_info.header.stamp = this->now();
                camera_info.width = image_msg_->width;
                camera_info.height = image_msg_->height;
                camera_info_manager_->setCameraInfo(camera_info);
                RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
            }
            camera_info_.header.frame_id = frame_id;
            camera_info_.header.stamp = this->now();

            // pub
            camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", rmw_qos_profile_sensor_data);

            // loop
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / max_fps), [this, frame_id]()
                                             {
            cap_ >> frame_;
            if (frame_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Video file ends!");
                if (!is_loop_) {
                rclcpp::shutdown();
                return;
                } else {
                cap_.open(video_path_);
                frame_cnt_ = 0;
                return;
                }
            }
            memcpy(image_msg_->data.data(), frame_.data, image_msg_->step * image_msg_->height);

            frame_cnt_++;
            if (frame_cnt_ < start_frame_)
            {
                RCLCPP_INFO(this->get_logger(), "Skip frame %d", frame_cnt_);
                return;
            }
            image_msg_->header.stamp = camera_info_.header.stamp = this->now();

            camera_pub_.publish(*image_msg_, camera_info_); });

            rclcpp::spin(this->get_node_base_interface());
        }
    };
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::VideoDriverNode)