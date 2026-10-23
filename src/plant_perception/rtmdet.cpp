#include "rtmdet.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
constexpr int kInputSize = 640;
constexpr int kBoxChannels = 4;
constexpr std::size_t kMaxDetections = 300;
constexpr std::array<int, 3> kStrides = {8, 16, 32};
constexpr std::array<float, 3> kBgrMeans = {103.53f, 116.28f, 123.675f};
constexpr std::array<float, 3> kBgrStandardDeviations = {57.375f, 57.12f, 58.395f};

bool has_shape(const cv::Mat& tensor, int channels, int grid_size)
{
    return tensor.dims == 4 && tensor.size[0] == 1 &&
        tensor.size[1] == channels && tensor.size[2] == grid_size &&
        tensor.size[3] == grid_size;
}
}  // namespace

RtmDet::RtmDet(
    const std::string& model_path,
    const std::string& mode,
    const std::vector<std::string>& class_names,
    double confidence_threshold,
    double nms_threshold)
    : model_path_(model_path),
      mode_(mode),
      class_names_(class_names),
      confidence_threshold_(confidence_threshold),
      nms_threshold_(nms_threshold),
      output_names_{
          "scores_s8", "distances_s8",
          "scores_s16", "distances_s16",
          "scores_s32", "distances_s32"}
{
    validate_parameters(confidence_threshold, nms_threshold);
    select_backend();
}

const std::string& RtmDet::backend() const
{
    return backend_;
}

const std::vector<std::string>& RtmDet::classes() const
{
    return class_names_;
}

void RtmDet::validate_parameters(double confidence_threshold, double nms_threshold) const
{
    if (mode_ != "auto" && mode_ != "cpu" && mode_ != "cuda") {
        throw std::invalid_argument("device must be auto, cpu, or cuda");
    }
    if (class_names_.empty()) {
        throw std::invalid_argument("class_names cannot be empty");
    }
    if (!std::isfinite(confidence_threshold) ||
        confidence_threshold <= 0.0 || confidence_threshold >= 1.0 ||
        !std::isfinite(nms_threshold) || nms_threshold < 0.0 || nms_threshold > 1.0) {
        throw std::invalid_argument(
            "confidence_threshold must be in (0,1), nms_threshold in [0,1]");
    }
}

void RtmDet::select_backend()
{
    if (mode_ == "cpu") {
        load_network(false);
        return;
    }

    try {
        const auto targets = cv::dnn::getAvailableTargets(cv::dnn::DNN_BACKEND_CUDA);
        if (std::find(targets.begin(), targets.end(), cv::dnn::DNN_TARGET_CUDA) ==
            targets.end()) {
            throw std::runtime_error(
                "OpenCV CUDA DNN target is unavailable "
                "(requires a CUDA-enabled OpenCV build and suitable GPU)");
        }
        load_network(true);
    } catch (const std::exception& error) {
        if (mode_ == "cuda") {
            throw;
        }
        std::cerr << "Automatic CPU fallback: " << error.what() << '\n';
        load_network(false);
    }
}

void RtmDet::load_network(bool use_cuda)
{
    network_ = cv::dnn::readNetFromONNX(model_path_);
    network_.setPreferableBackend(
        use_cuda ? cv::dnn::DNN_BACKEND_CUDA : cv::dnn::DNN_BACKEND_OPENCV);
    network_.setPreferableTarget(
        use_cuda ? cv::dnn::DNN_TARGET_CUDA : cv::dnn::DNN_TARGET_CPU);

    // A successful warm-up verifies execution, not merely GPU availability.
    const int shape[] = {1, 3, kInputSize, kInputSize};
    network_.setInput(cv::Mat(4, shape, CV_32F, cv::Scalar(0)));
    std::vector<cv::Mat> outputs;
    network_.forward(outputs, output_names_);
    validate_outputs(outputs);

    backend_ = use_cuda ? "cuda" : "cpu";
}

void RtmDet::validate_outputs(const std::vector<cv::Mat>& outputs) const
{
    if (outputs.size() != kStrides.size() * 2) {
        throw std::runtime_error("Expected six RTMDet outputs");
    }

    for (std::size_t level = 0; level < kStrides.size(); ++level) {
        const int grid_size = kInputSize / kStrides[level];
        const auto& logits = outputs[level * 2];
        const auto& distances = outputs[level * 2 + 1];
        if (!has_shape(logits, static_cast<int>(class_names_.size()), grid_size) ||
            !has_shape(distances, kBoxChannels, grid_size)) {
            throw std::runtime_error(
                "Model output shape does not match configured classes and 640x640 input");
        }
    }
}

RtmDet::PreparedInput RtmDet::prepare_input(const cv::Mat& image)
{
    if (image.empty() || image.type() != CV_8UC3) {
        throw std::invalid_argument("Expected nonempty BGR8 image");
    }

    const double scale = static_cast<double>(kInputSize) /
        std::max(image.cols, image.rows);
    const cv::Size resized_size(
        static_cast<int>(image.cols * scale + 0.5),
        static_cast<int>(image.rows * scale + 0.5));

    cv::Mat resized;
    cv::resize(image, resized, resized_size, 0, 0, cv::INTER_LINEAR);

    // Match MMDetection: preserve aspect ratio and pad only bottom/right.
    cv::Mat padded(kInputSize, kInputSize, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(cv::Point(0, 0), resized_size)));
    padded.convertTo(padded, CV_32FC3);

    std::vector<cv::Mat> channels;
    cv::split(padded, channels);
    for (std::size_t channel = 0; channel < channels.size(); ++channel) {
        channels[channel] = (channels[channel] - kBgrMeans[channel]) /
            kBgrStandardDeviations[channel];
    }
    cv::merge(channels, padded);

    // Input is already normalized BGR; do not swap channels or normalize again.
    const auto blob = cv::dnn::blobFromImage(
        padded, 1.0, cv::Size(), cv::Scalar(), false, false);
    return {blob, resized_size, image.size()};
}

std::vector<Detection> RtmDet::infer(const cv::Mat& image)
{
    try {
        return infer_once(image);
    } catch (const cv::Exception& error) {
        if (mode_ != "auto" || backend_ != "cuda") {
            throw;
        }
        std::cerr << "CUDA runtime failure: " << error.what()
                  << "; retrying on CPU\n";
        load_network(false);
        return infer_once(image);
    }
}

std::vector<Detection> RtmDet::infer_once(const cv::Mat& image)
{
    const auto input = prepare_input(image);
    network_.setInput(input.blob);

    std::vector<cv::Mat> outputs;
    network_.forward(outputs, output_names_);
    validate_outputs(outputs);

    return apply_nms(decode_outputs(outputs, input));
}

std::vector<Detection> RtmDet::decode_outputs(
    const std::vector<cv::Mat>& outputs,
    const PreparedInput& input) const
{
    std::vector<Detection> candidates;
    for (std::size_t level = 0; level < kStrides.size(); ++level) {
        decode_level(
            outputs[level * 2], outputs[level * 2 + 1],
            kStrides[level], input, candidates);
    }
    return candidates;
}

void RtmDet::decode_level(
    const cv::Mat& logits,
    const cv::Mat& distances,
    int stride,
    const PreparedInput& input,
    std::vector<Detection>& candidates) const
{
    const int grid_size = kInputSize / stride;
    const int plane_size = grid_size * grid_size;
    const auto* score_data = logits.ptr<float>();
    const auto* distance_data = distances.ptr<float>();

    for (int label = 0; label < static_cast<int>(class_names_.size()); ++label) {
        for (int row = 0; row < grid_size; ++row) {
            for (int column = 0; column < grid_size; ++column) {
                const int index = row * grid_size + column;
                const float logit = score_data[label * plane_size + index];
                const float confidence = 1.0f / (1.0f + std::exp(-logit));
                if (!std::isfinite(confidence) || confidence < confidence_threshold_) {
                    continue;
                }

                // Point offset is zero; exported distances already include stride.
                const cv::Point point(column * stride, row * stride);
                const auto box = decode_box(
                    distance_data, index, plane_size, point, input);
                if (std::isfinite(box.x + box.y + box.width + box.height) &&
                    box.width > 0.0 && box.height > 0.0) {
                    candidates.push_back({label, confidence, box});
                }
            }
        }
    }
}

cv::Rect2d RtmDet::decode_box(
    const float* distances,
    int index,
    int plane_size,
    const cv::Point& point,
    const PreparedInput& input)
{
    const double width = input.resized_size.width;
    const double height = input.resized_size.height;
    const double scale_x = width / input.original_size.width;
    const double scale_y = height / input.original_size.height;

    // Clip to the actual resized image, excluding padding, then undo resize.
    const double x1 = std::clamp(point.x - static_cast<double>(distances[index]),
        0.0, width) / scale_x;
    const double y1 = std::clamp(point.y - static_cast<double>(distances[plane_size + index]),
        0.0, height) / scale_y;
    const double x2 = std::clamp(point.x + static_cast<double>(distances[2 * plane_size + index]),
        0.0, width) / scale_x;
    const double y2 = std::clamp(point.y + static_cast<double>(distances[3 * plane_size + index]),
        0.0, height) / scale_y;

    return {x1, y1, x2 - x1, y2 - y1};
}

std::vector<Detection> RtmDet::apply_nms(const std::vector<Detection>& candidates) const
{
    std::vector<Detection> detections;
    for (int label = 0; label < static_cast<int>(class_names_.size()); ++label) {
        std::vector<cv::Rect2d> boxes;
        std::vector<float> confidences;
        std::vector<std::size_t> candidate_indices;

        // Suppress overlaps within each class, never between Weed and Plant.
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            if (candidates[index].label == label) {
                boxes.push_back(candidates[index].box);
                confidences.push_back(candidates[index].score);
                candidate_indices.push_back(index);
            }
        }

        std::vector<int> kept_indices;
        cv::dnn::NMSBoxes(
            boxes, confidences, confidence_threshold_, nms_threshold_, kept_indices);
        for (const int index : kept_indices) {
            detections.push_back(candidates[candidate_indices[index]]);
        }
    }

    std::sort(detections.begin(), detections.end(),
        [](const Detection& left, const Detection& right) {
            return left.score > right.score;
        });
    if (detections.size() > kMaxDetections) {
        detections.resize(kMaxDetections);
    }
    return detections;
}
