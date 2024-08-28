#include "detector_cpp/detector_node.hpp"
#include <cv_bridge/cv_bridge.h>

namespace driver_detector
{
    DetectorNode::DetectorNode(rclcpp::NodeOptions options) : Node("DetectorNode", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

        initTrafficSignDetector();
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS(), std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1));
        result_img_pub_ = image_transport::create_publisher(this, "signs_result", rmw_qos_profile_sensor_data);
        image_encodings_=this->declare_parameter("image_encodings","rgb8");
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
                                                                                {
            cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info); 
            cam_info_sub_.reset(); });
    }

    void DetectorNode::initTrafficSignDetector()
    {
        std::string model_path = this->declare_parameter("detector.traffic_sign.model_path", "package://resource/models/traffic_sign_model/best.xml");
        std::string device_type = this->declare_parameter("detector.device_type", "CPU");
        std::string metadata_path = this->declare_parameter("detector.metadata_path", "package://resource/models/traffic_sign_model/metadata.yaml");
        RCLCPP_INFO(this->get_logger(), "Createing TrafficSignDetector! ");
        RCLCPP_INFO(this->get_logger(), "model: %s, device: %s", model_path.c_str(), device_type.c_str());
        float conf_threshold = this->declare_parameter("detector.traffic_sign.confidence_threshold", 0.50);
        traffic_sign_detector_ = std::make_unique<TrafficSignDetector>(
            utils::URLResolver::getPackageFileName(model_path), device_type,
            utils::URLResolver::getPackageFileName(metadata_path), conf_threshold);
    }

    std::vector<TrafficSign> DetectorNode::detectSigns(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        auto img = cv_bridge::toCvCopy(msg, image_encodings_)->image;
        auto signs = traffic_sign_detector_->runDetect(img);

        auto final_time = this->now();
        auto latency = (final_time - msg->header.stamp).seconds() * 1000;

        traffic_sign_detector_->drawResult(img);
        cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
        std::stringstream latency_ss;
        latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
        auto latency_s = latency_ss.str();
        cv::putText(
            img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        result_img_pub_.publish(cv_bridge::CvImage(msg->header, image_encodings_, img).toImageMsg());
        return signs;
    }

    void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        auto timestamp = rclcpp::Time(msg->header.stamp);
        auto signs = detectSigns(msg);
    }
} //
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(driver_detector::DetectorNode)