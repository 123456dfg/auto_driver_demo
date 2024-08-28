#ifndef _TRAFFIC_SIGN_DETECTOR_H_
#define _TRAFFIC_SIGN_DETECTOR_H_

#include "detector_cpp/traffic_sign.hpp"
#include <unordered_map>
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include "utils/url_resolver.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <random>
namespace driver_detector
{
    
    class TrafficSignDetector
    {
    private:

        void Preprocessing(const cv::Mat &frame,TransformStruct &transform);
        static cv::Mat letterbox(const cv::Mat &img,TransformStruct &transform);
        
        void PostProcessing(TransformStruct &transform);
        void initModel();
        
        std::unique_ptr<ov::Core> ov_core_;
        std::unique_ptr<ov::CompiledModel> compiled_model_;
        ov::InferRequest inference_request_;
        std::string model_path_;
        std::string device_name_;
        std::vector<TrafficSign> detections_;
        cv::Size model_output_shape_;
        float conf_threshold_;
        std::vector<std::string>class_names;

    public:
        explicit TrafficSignDetector(const std::string &model_path, const std::string &device,const std::string& classed_path,float conf_threshold=0.5);
        std::vector<TrafficSign> runDetect(cv::Mat &img);
        void drawResult(cv::Mat &src);
    };

} // namespace driver_detector

#endif // _TRAFFIC_SIGN_DETECTOR_H_