#include "detector_cpp/traffic_sign_detector.hpp"

namespace driver_detector
{
    static constexpr int IMG_SIZE = 480;

    void TrafficSignDetector::Preprocessing(const cv::Mat &frame, TransformStruct &transform)
    {
        cv::Mat resized_frame = letterbox(frame, transform);
        float *input_data = (float *)resized_frame.data;
        const ov::Tensor input_tensor = ov::Tensor(compiled_model_->input().get_element_type(), compiled_model_->input().get_shape(), input_data);
        inference_request_.set_input_tensor(input_tensor);
    }

    cv::Mat TrafficSignDetector::letterbox(const cv::Mat &img, TransformStruct &transform)
    {
        int img_h = img.rows;
        int img_w = img.cols;

        // Compute scale ratio(new / old) and target resized shape
        float scale = std::min(IMG_SIZE * 1.0 / img_h, IMG_SIZE * 1.0 / img_w);
        int resize_h = static_cast<int>(round(img_h * scale));
        int resize_w = static_cast<int>(round(img_w * scale));

        // Compute padding
        int pad_h = IMG_SIZE - resize_h;
        int pad_w = IMG_SIZE - resize_w;

        // Resize and pad image while meeting stride-multiple constraints
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(resize_w, resize_h));

        // divide padding into 2 sides
        float half_h = pad_h * 1.0 / 2;
        float half_w = pad_w * 1.0 / 2;

        transform = TransformStruct(1.0f / scale, half_h, half_w);
        // Compute padding boarder
        int top = static_cast<int>(round(half_h - 0.1));
        int bottom = static_cast<int>(round(half_h + 0.1));
        int left = static_cast<int>(round(half_w - 0.1));
        int right = static_cast<int>(round(half_w + 0.1));

        // Add border
        cv::copyMakeBorder(resized_img,
                           resized_img,
                           top,
                           bottom,
                           left,
                           right,
                           cv::BORDER_CONSTANT,
                           cv::Scalar(114, 114, 114));

        return resized_img;
    }
    void TrafficSignDetector::PostProcessing(TransformStruct &transform)
    {
        const float *detections = inference_request_.get_output_tensor().data<const float>();
        detections_.clear();

        /*
         * 0  1  2  3      4          5
         * x, y, w. h, confidence, class_id
         */

        for (int i = 0; i < model_output_shape_.height; ++i)
        {
            const int index = i * model_output_shape_.width;

            const float &confidence = detections[index + 4];

            if (confidence > conf_threshold_)
            {
                float x = detections[index + 0];
                float y = detections[index + 1];
                float w = detections[index + 2] - x;
                float h = detections[index + 3] - y;

                TrafficSign result;

                result.class_id = static_cast<const short>(detections[index + 5]);
                result.confidence = confidence;
                float &scale = transform.scale_, &half_w = transform.hw_, &half_h = transform.hh_;
                w *= scale;
                h *= scale;

                x = x * scale - half_w * scale;
                y = y * scale - half_h * scale;

                result.box = cv::Rect(x, y, w, h);
                detections_.push_back(result);
            }
        }
    }
    void TrafficSignDetector::initModel()
    {
        if (ov_core_ == nullptr)
        {
            ov_core_ = std::make_unique<ov::Core>();
        }

        auto model = ov_core_->read_model(model_path_);

        if (model->is_dynamic())
        {
            model->reshape({1, 3, static_cast<long int>(IMG_SIZE), static_cast<long int>(IMG_SIZE)});
        }

        // Set infer type
        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
        ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({255, 255, 255});
        ppp.input().model().set_layout("NCHW");

        model = ppp.build();

        auto perf_mode = ov::hint::performance_mode(device_name_ == "GPU" ? ov::hint::PerformanceMode::THROUGHPUT : ov::hint::PerformanceMode::LATENCY);
        compiled_model_ =
            std::make_unique<ov::CompiledModel>(ov_core_->compile_model(model, device_name_, perf_mode));

        inference_request_ = compiled_model_->create_infer_request();

        const std::vector<ov::Output<ov::Node>> outputs = model->outputs();
        const ov::Shape output_shape = outputs[0].get_shape();
        model_output_shape_ = cv::Size(output_shape[1], output_shape[2]);
    }
    void TrafficSignDetector::drawResult(cv::Mat &src)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(120, 255);

        for (const auto &detection : detections_)
        {
            const cv::Rect &box = detection.box;
            const float &confidence = detection.confidence;
            const int &class_id = detection.class_id;

            const cv::Scalar color = cv::Scalar(dis(gen), dis(gen), dis(gen));
            cv::rectangle(src, box, color, 10);

            std::string class_string;

            if (class_names.empty())
                class_string = "id[" + std::to_string(class_id) + "] " + std::to_string(confidence).substr(0, 4);
            else
                class_string = class_names[class_id] + " " + std::to_string(confidence).substr(0, 4);

            const cv::Size text_size = cv::getTextSize(class_string, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, 0);
            const cv::Rect text_box(box.x - 2, box.y - 27, text_size.width + 10, text_size.height + 15);

            cv::rectangle(src, text_box, color, cv::FILLED);
            cv::putText(src, class_string, cv::Point(box.x + 5, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2, 0);
        }
        return;
    }

    TrafficSignDetector::TrafficSignDetector(const std::string &model_path, const std::string &device, const std::string &classed_path, float conf_threshold)
        : model_path_(model_path), device_name_(device), conf_threshold_(conf_threshold)
    {

        initModel();
        class_names = utils::URLResolver::getClassNameFromMetadata(classed_path);
    }
    std::vector<TrafficSign> TrafficSignDetector::runDetect(cv::Mat &img)
    {

        TransformStruct transform;
        Preprocessing(img, transform);
        inference_request_.infer();
        PostProcessing(transform);
        return detections_;
    }
}
