#ifndef SONAR_H_
#define SONAR_H_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rs1/srv/get_cube_centre.hpp"

class SonarNode : public rclcpp::Node
{
public:
  SonarNode();

private:

  void getCubeCentre(
  const std::shared_ptr<rs1::srv::GetCubeCentre::Request> request,
  std::shared_ptr<rs1::srv::GetCubeCentre::Response> response);

  double cube_search_radius_{1.5};  // how far from the camera axis to look for cube points
  double cube_max_age_{1.0};        // s; older detections are treated as "no cube"

  struct CubeCentre
  {
    bool valid{false};
    double forward{0.0};
    double left{0.0};
    double depth{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };
  CubeCentre latest_cube_;

  rclcpp::Service<rs1::srv::GetCubeCentre>::SharedPtr cube_service_;

  
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::string cloud_topic_;
  double max_lateral_distance_{1.0};
  double centre_radius_{0.4};       // narrow window under the glider, used for the cube top
  double min_range_{0.45};
  double max_range_{12.0};
  double high_point_margin_{0.08};  // m closer than floor => "high"
  int min_points_{30};
  int min_high_points_{10};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr floor_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr high_bool_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr high_range_pub_;
};

#endif  // SONAR_H_