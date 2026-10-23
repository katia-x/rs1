#ifndef RS1_PLANT_PERCEPTION_PLANT_DETECTOR_H
#define RS1_PLANT_PERCEPTION_PLANT_DETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cv {
class Mat;
}

class RtmDet;
struct Detection;

// Retains the newest camera frame and publishes timestamped inference results.
class PlantDetector : public rclcpp::Node {
public:
    PlantDetector();
    ~PlantDetector() override;

private:
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr message);
    void process_latest_image();
    void publish_results(
        const sensor_msgs::msg::Image& input,
        cv::Mat& image,
        const std::vector<Detection>& results);
    vision_msgs::msg::Detection2D make_detection_message(
        const Detection& result,
        const std_msgs::msg::Header& header) const;
    static void draw_detection(
        cv::Mat& image,
        const Detection& result,
        const std::string& class_name);

    std::unique_ptr<RtmDet> engine_;

    // Protect the image/header pair while acquisition and inference run separately.
    std::mutex image_mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_image_;

    rclcpp::CallbackGroup::SharedPtr camera_group_;
    rclcpp::CallbackGroup::SharedPtr inference_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_publisher_;
    rclcpp::TimerBase::SharedPtr inference_timer_;
};

#endif  // RS1_PLANT_PERCEPTION_PLANT_DETECTOR_H
