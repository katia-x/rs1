/*ToDO:
Subscribe to /glider/sonar topic
*/

#include "glider_node.h"

#include <chrono>
#include <functional>

using std::placeholders::_1;


GliderNode::GliderNode()
: Node("glider_node")
{

  // ---------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------

  this->declare_parameter("home_x", 0.0);
  this->declare_parameter("home_y", 0.0);
  this->declare_parameter("home_z", 1.5);

  this->declare_parameter("safe_height", 15.0);
  this->declare_parameter("pickup_height", 1.5);

  this->declare_parameter("xy_tolerance", 0.4);
  this->declare_parameter("z_tolerance", 0.15);

  this->declare_parameter("control_rate", 20.0);


  // ---------------------------------------------------------
  // Research facility/home position
  // ---------------------------------------------------------

  glider::Pose home;

  home.x =
    this->get_parameter("home_x").as_double();

  home.y =
    this->get_parameter("home_y").as_double();

  home.z =
    this->get_parameter("home_z").as_double();


  // ---------------------------------------------------------
  // Create processing object
  // ---------------------------------------------------------

  processing_ =
    std::make_unique<glider::GliderProcessing>(
      home,
      this->get_parameter("safe_height").as_double(),
      this->get_parameter("pickup_height").as_double(),
      this->get_parameter("xy_tolerance").as_double(),
      this->get_parameter("z_tolerance").as_double());


  // ---------------------------------------------------------
  // Poo location input
  // ---------------------------------------------------------
  //
  // One PoseStamped message = one poo location.
  //
  // Eventually Mission will publish these.
  //
  // For now you can publish them manually from the terminal.
  //
  traj_sub_ =
    this->create_subscription<
      geometry_msgs::msg::PoseStamped>(
        "/glider/trajectory_goals",
        10,
        std::bind(
          &GliderNode::trajectoryCallback,
          this,
          _1));


  // ---------------------------------------------------------
  // Parrot odometry
  // ---------------------------------------------------------
  //
  // This is provided by the RS1 Parrot simulator.
  //
  // Actual topic:
  //
  //     /parrot1/odometry
  //
  odom_sub_ =
    this->create_subscription<
      nav_msgs::msg::Odometry>(
        "/parrot1/odometry",
        20,
        std::bind(
          &GliderNode::odometryCallback,
          this,
          _1));


  // ---------------------------------------------------------
  // Height sensor
  // ---------------------------------------------------------
  //
  // Future sensor input.
  //
  // The current Parrot model does not actually publish
  // sensor_msgs/Range, so the processing layer will use
  // odometry as a temporary fallback.
  //
  height_sub_ =
    this->create_subscription<
      sensor_msgs::msg::Range>(
        "/glider/height_above_poo",
        10,
        std::bind(
          &GliderNode::heightCallback,
          this,
          _1));


  // ---------------------------------------------------------
  // Parrot velocity command
  // ---------------------------------------------------------
  //
  // The RS1 Parrot accepts:
  //
  //     /parrot1/cmd_vel
  //
  // as geometry_msgs/Twist.
  //
  cmd_vel_pub_ =
    this->create_publisher<
      geometry_msgs::msg::Twist>(
        "/parrot1/cmd_vel",
        10);


  // ---------------------------------------------------------
  // Status output
  // ---------------------------------------------------------

  status_pub_ =
    this->create_publisher<
      std_msgs::msg::String>(
        "/glider/status",
        10);


  // ---------------------------------------------------------
  // Control loop
  // ---------------------------------------------------------

  const double rate =
    this->get_parameter("control_rate").as_double();

  timer_ =
    this->create_wall_timer(
      std::chrono::duration_cast<
        std::chrono::milliseconds>(
          std::chrono::duration<double>(
            1.0 / rate)),
      std::bind(
        &GliderNode::controlLoop,
        this));


  RCLCPP_INFO(
    this->get_logger(),
    "Glider node ready - waiting for poo locations");
}


/*
 * ---------------------------------------------------------
 * Receive poo location
 * ---------------------------------------------------------
 */
void GliderNode::trajectoryCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  glider::Goal goal;

  goal.pose.x =
    msg->pose.position.x;

  goal.pose.y =
    msg->pose.position.y;

  goal.pose.z =
    msg->pose.position.z;

  goal.pose.yaw = 0.0;

  goal.is_poo = true;


  /*
   * Add the goal rather than replacing the previous goals.
   *
   * This allows:
   *
   * poo 1
   * poo 2
   * poo 3
   *
   * to be queued.
   */
  processing_->addPooGoal(goal);


  RCLCPP_INFO(
    this->get_logger(),
    "Received poo location: x=%.2f y=%.2f z=%.2f",
    goal.pose.x,
    goal.pose.y,
    goal.pose.z);
}


/*
 * ---------------------------------------------------------
 * Receive Parrot odometry
 * ---------------------------------------------------------
 */
void GliderNode::odometryCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  glider::Pose pose;

  pose.x =
    msg->pose.pose.position.x;

  pose.y =
    msg->pose.pose.position.y;

  pose.z =
    msg->pose.pose.position.z;

  const auto & q = msg->pose.pose.orientation;

  pose.yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));


  processing_->setPose(pose);
}


/*
 * ---------------------------------------------------------
 * Receive height sensor
 * ---------------------------------------------------------
 */
void GliderNode::heightCallback(
  const sensor_msgs::msg::Range::SharedPtr msg)
{
  if (
    msg->range >= msg->min_range &&
    msg->range <= msg->max_range)
  {
    processing_->setHeightAbovePoo(
      msg->range);
  }
}


/*
 * ---------------------------------------------------------
 * Control loop
 * ---------------------------------------------------------
 *
 * Notice that there is NO flight logic here.
 *
 * The node simply:
 *
 * 1. asks processing for a command
 * 2. converts that command into a ROS Twist
 * 3. publishes it
 */
void GliderNode::controlLoop()
{
  const auto [command, status] =
    processing_->tick();


  // Convert processing command to ROS message
  geometry_msgs::msg::Twist cmd;

  cmd.linear.x =
    command.vx;

  cmd.linear.y =
    command.vy;

  cmd.linear.z =
    command.vz;

  cmd.angular.z =
    command.yaw_rate;


  // Send command to Parrot
  cmd_vel_pub_->publish(cmd);


  // Publish state/status
  if (status != last_status_) {

  std_msgs::msg::String status_msg;

  status_msg.data = status;

  status_pub_->publish(status_msg);

  last_status_ = status;
}
}


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<GliderNode>());

  rclcpp::shutdown();

  return 0;
}