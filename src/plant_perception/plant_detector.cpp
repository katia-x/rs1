#include "plant_detector.h"
#include "rtmdet.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

PlantDetector::PlantDetector()
    : Node("plant_detector")
{
    const auto model_path = declare_parameter<std::string>("model_path", "");
    const auto device = declare_parameter<std::string>("device", "auto");
    const auto reliability = declare_parameter<std::string>(
        "camera_reliability", "best_effort");
    const auto camera_topic = declare_parameter<std::string>(
        "camera_topic", "/husky1/camera/image");
    const auto annotated_topic = declare_parameter<std::string>(
        "annotated_topic", "/husky1/detections/image");
    const auto detections_topic = declare_parameter<std::string>(
        "detections_topic", "/husky1/detections");
    const auto class_names = declare_parameter<std::vector<std::string>>(
        "class_names", {"Weed", "Plant"});
    const double confidence_threshold = declare_parameter<double>(
        "confidence_threshold", 0.3);
    const double nms_threshold = declare_parameter<double>("nms_threshold", 0.65);
    const double inference_rate_hz = declare_parameter<double>("inference_rate_hz", 2.0);

    if (reliability != "best_effort" && reliability != "reliable") {
        throw std::invalid_argument(
            "camera_reliability must be best_effort or reliable");
    }
    if (!std::isfinite(inference_rate_hz) ||
        inference_rate_hz <= 0.0 || inference_rate_hz > 1000.0) {
        throw std::invalid_argument("inference_rate_hz must be in (0,1000]");
    }

    engine_ = std::make_unique<RtmDet>(
        model_path, device, class_names, confidence_threshold, nms_threshold);

    RCLCPP_INFO(
        get_logger(), "Requested device=%s; selected backend=%s",
        device.c_str(), engine_->backend().c_str());
    if (device == "auto" && engine_->backend() == "cpu") {
        RCLCPP_INFO(get_logger(), "CUDA unavailable or failed warm-up; using CPU");
    }

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic, 5);
    detection_publisher_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        detections_topic, 5);

    // Inference may block; keep camera acquisition in its own callback group.
    camera_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    inference_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = camera_group_;
    auto camera_qos = rclcpp::SensorDataQoS().keep_last(1);
    if (reliability == "reliable") {
        camera_qos.reliable();
    }

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
        camera_topic, camera_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
            image_callback(std::move(message));
        },
        subscription_options);

    const auto inference_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / inference_rate_hz));
    inference_timer_ = create_wall_timer(
        inference_period,
        [this]() { process_latest_image(); },
        inference_group_);
}

// Defined where RtmDet is complete, so the header can forward-declare it.
PlantDetector::~PlantDetector() = default;

void PlantDetector::image_callback(sensor_msgs::msg::Image::ConstSharedPtr message)
{
    RCLCPP_DEBUG(get_logger(), "Received frame %d", message->header.stamp.sec);

    std::lock_guard<std::mutex> guard(image_mutex_);
    latest_image_ = std::move(message);
}

void PlantDetector::process_latest_image()
{
    sensor_msgs::msg::Image::ConstSharedPtr message;
    {
        std::lock_guard<std::mutex> guard(image_mutex_);
        message.swap(latest_image_);
    }
    if (!message) {
        return;
    }

    RCLCPP_DEBUG(get_logger(), "Processing frame %d", message->header.stamp.sec);

    try {
        auto image = cv_bridge::toCvCopy(message, "bgr8")->image;
        const auto previous_backend = engine_->backend();
        const auto results = engine_->infer(image);

        if (previous_backend != engine_->backend()) {
            RCLCPP_WARN(get_logger(), "CUDA inference failed; switched to CPU");
        }

        publish_results(*message, image, results);
    } catch (const std::exception& error) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Frame processing failed: %s", error.what());
    }
}

void PlantDetector::publish_results(
    const sensor_msgs::msg::Image& input,
    cv::Mat& image,
    const std::vector<Detection>& results)
{
    vision_msgs::msg::Detection2DArray detections;
    detections.header = input.header;
    detections.detections.reserve(results.size());

    for (const auto& result : results) {
        detections.detections.push_back(make_detection_message(result, input.header));
        draw_detection(image, result, engine_->classes().at(result.label));
    }

    // Preserve acquisition time even when inference finishes much later.
    detection_publisher_->publish(detections);
    image_publisher_->publish(
        *cv_bridge::CvImage(input.header, "bgr8", image).toImageMsg());
}

vision_msgs::msg::Detection2D PlantDetector::make_detection_message(
    const Detection& result,
    const std_msgs::msg::Header& header) const
{
    vision_msgs::msg::Detection2D detection;
    detection.header = header;

    const auto& box = result.box;
    detection.bbox.center.position.x = box.x + box.width / 2.0;
    detection.bbox.center.position.y = box.y + box.height / 2.0;
    detection.bbox.size_x = box.width;
    detection.bbox.size_y = box.height;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = engine_->classes().at(result.label);
    hypothesis.hypothesis.score = result.score;
    detection.results.push_back(hypothesis);

    return detection;
}

void PlantDetector::draw_detection(
    cv::Mat& image,
    const Detection& result,
    const std::string& class_name)
{
    const auto& box = result.box;
    const cv::Point top_left(cvRound(box.x), cvRound(box.y));
    const cv::Point bottom_right(
        cvRound(box.x + box.width), cvRound(box.y + box.height));
    const cv::Point label_position(
        std::max(0, top_left.x), std::max(20, top_left.y - 10));
    const cv::Scalar colour(0, 255, 0);

    cv::rectangle(image, top_left, bottom_right, colour, 2);
    cv::putText(
        image, cv::format("%s %.3f", class_name.c_str(), result.score),
        label_position, cv::FONT_HERSHEY_SIMPLEX,
        0.5, colour, 1, cv::LINE_AA);
}
