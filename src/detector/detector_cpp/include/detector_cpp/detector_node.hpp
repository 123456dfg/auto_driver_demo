#ifndef _DETECTOR_NODE_H_
#define _DETECTOR_NODE_H_

// ros2
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// opencv
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
// std
#include <memory>

// utils
#include "utils/url_resolver.hpp"
#include "detector_cpp/traffic_sign_detector.hpp"

namespace driver_detector
{
    class DetectorNode : public rclcpp::Node
    {
    private:
        void initTrafficSignDetector();
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        std::vector<TrafficSign> detectSigns(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

        std::unique_ptr<TrafficSignDetector> traffic_sign_detector_;
        std::string frame_id_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        image_transport::Publisher result_img_pub_;

        // camera info part
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
        cv::Point2f cam_center_;
        std::string image_encodings_;
    public:
        DetectorNode(rclcpp::NodeOptions options);
    };

} // namespace driver_detector

#endif // _DETECTOR_NODE_H_