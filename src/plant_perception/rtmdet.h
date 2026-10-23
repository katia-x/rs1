#ifndef RS1_PLANT_PERCEPTION_RTMDET_H
#define RS1_PLANT_PERCEPTION_RTMDET_H

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

struct Detection {
    int label;
    float score;
    cv::Rect2d box;  // Original camera-image coordinates.
};

// OpenCV inference for the exported RTMDet model; independent of ROS.
class RtmDet {
public:
    RtmDet(
        const std::string& model_path,
        const std::string& mode,
        const std::vector<std::string>& class_names,
        double confidence_threshold,
        double nms_threshold);

    const std::string& backend() const;
    const std::vector<std::string>& classes() const;
    std::vector<Detection> infer(const cv::Mat& image);

private:
    struct PreparedInput {
        cv::Mat blob;
        cv::Size resized_size;
        cv::Size original_size;
    };

    void validate_parameters(double confidence_threshold, double nms_threshold) const;
    void select_backend();
    void load_network(bool use_cuda);
    void validate_outputs(const std::vector<cv::Mat>& outputs) const;

    static PreparedInput prepare_input(const cv::Mat& image);
    std::vector<Detection> infer_once(const cv::Mat& image);
    std::vector<Detection> decode_outputs(
        const std::vector<cv::Mat>& outputs,
        const PreparedInput& input) const;
    void decode_level(
        const cv::Mat& logits,
        const cv::Mat& distances,
        int stride,
        const PreparedInput& input,
        std::vector<Detection>& candidates) const;
    static cv::Rect2d decode_box(
        const float* distances,
        int index,
        int plane_size,
        const cv::Point& point,
        const PreparedInput& input);
    std::vector<Detection> apply_nms(const std::vector<Detection>& candidates) const;

    std::string model_path_;
    std::string mode_;
    std::string backend_;
    std::vector<std::string> class_names_;
    float confidence_threshold_;
    float nms_threshold_;
    cv::dnn::Net network_;
    std::vector<cv::String> output_names_;
};

#endif  // RS1_PLANT_PERCEPTION_RTMDET_H
