/*ToDO:
Subscribe to /glider/sonar topic
*/

#include "glider_node.h"

#include <chrono>
#include <functional>

using std::placeholders::_1;


GliderNode::GliderNode()
: Node("glider_node"),
  platform_(this->get_logger())
{

  // ---------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------

  this->declare_parameter("home_x", -17.5);
  this->declare_parameter("home_y", 13.0);
  this->declare_parameter("home_z", 0.2);

  this->declare_parameter("truck_x", -16.5);
  this->declare_parameter("truck_y", 16.5);
  this->declare_parameter("truck_z", 2.0);

  this->declare_parameter("safe_height", 15.0);
  this->declare_parameter("pickup_height", 0.2);
  this->declare_parameter("align_height", 3.0);

  this->declare_parameter("xy_tolerance", 0.4);
  this->declare_parameter("z_tolerance", 0.15);

  this->declare_parameter("control_rate", 20.0);

  this->declare_parameter("align_tolerance", 0.08);


  // ---------------------------------------------------------
  // Home position
  // ---------------------------------------------------------

  glider::Pose home;

  home.x =
    this->get_parameter("home_x").as_double();

  home.y =
    this->get_parameter("home_y").as_double();

  home.z =
    this->get_parameter("home_z").as_double();

  // ---------------------------------------------------------
  // Truck position
  // ---------------------------------------------------------

  glider::Pose truck;

  truck.x =
    this->get_parameter("truck_x").as_double();

  truck.y =
    this->get_parameter("truck_y").as_double();

  truck.z =
    this->get_parameter("truck_z").as_double();


  // ---------------------------------------------------------
  // Create processing object
  // ---------------------------------------------------------

  processing_ =
    std::make_unique<glider::GliderProcessing>(
      home, truck,
      this->get_parameter("safe_height").as_double(),
      this->get_parameter("pickup_height").as_double(),
      this->get_parameter("align_height").as_double(),
      this->get_parameter("xy_tolerance").as_double(),
      this->get_parameter("z_tolerance").as_double());

  processing_->setAlignTolerance(this->get_parameter("align_tolerance").as_double());
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
      rs1::msg::GliderGoal>(
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
  // Cube centre
  // ---------------------------------------------------------
  //
  // Server client to ask for poo centre.
  //
  sonar_client_ = 
    this->create_client<
    rs1::srv::GetCubeCentre>(
      "/glider/sonar");

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
void GliderNode::trajectoryCallback(const rs1::msg::GliderGoal::SharedPtr msg)
{
  glider::Goal goal;

  goal.pose.x =
    msg->x;

  goal.pose.y =
    msg->y;

  goal.pose.z =
    msg->z;

  goal.pose.yaw = 0.0;

  goal.is_poo = true;

  goal.model_name = 
    msg->model_name;


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
    "Received poo location: name=%s x=%.2f y=%.2f z=%.2f",
    goal.model_name.c_str(),
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

  current_pose_ = pose;
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

// ---------------------------------------------------------
// Remove a poo model from the world
// ---------------------------------------------------------
void GliderNode::removePooModel(const std::string & name)
{
  // Using ignition
  std::stringstream ss;
  ss << "ign service -s /world/rs1_terrain/remove "
     << "--reqtype ignition.msgs.Entity "
     << "--reptype ignition.msgs.Boolean "
     << "--timeout 3000 "
     << "--req 'name: \"" << name << "\", type: MODEL'";  

  int ret = std::system(ss.str().c_str());
  if (ret != 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to remove model '%s'", name.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Removed model '%s'", name.c_str());
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

  // Detect pickup
  static bool was_carrying = false;
  bool is_carrying = processing_->isCarryingPoo();   // add a simple getter

  if (is_carrying && !was_carrying) {
    const std::string name = processing_->currentModelName();
    if (!name.empty()) {
      removePooModel(name);
      current_model_name_ = name;   // remember it for the drop
    }
  }

  if (!is_carrying && was_carrying) {
    if (!current_model_name_.empty()) {
      platform_.spawnPoo(current_model_name_, current_pose_.x, current_pose_.y, current_pose_.z - 0.20);
      current_model_name_.clear();
    }
  }

  was_carrying = is_carrying;
    // Ask the sonar for the cube centre whenever ALIGN wants a reading
  if (processing_->wantsCubeCentre()) {
    if (sonar_client_->service_is_ready()) {
      processing_->markCubeCentreRequested();

      auto request = std::make_shared<rs1::srv::GetCubeCentre::Request>();
      const glider::Pose pose_at_request = current_pose_;

      sonar_client_->async_send_request(
        request,
        [this, pose_at_request](rclcpp::Client<rs1::srv::GetCubeCentre>::SharedFuture future)
        {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_WARN(this->get_logger(), "Sonar service: %s", response->message.c_str());
            processing_->setCubeCentreUnavailable();
            return;
          }

          // Service gives an offset from the glider (forward, left, depth);
          // convert to world using the pose from when we asked.
          const double fwd = response->centre.position.x;
          const double left = response->centre.position.y;
          const double c = std::cos(pose_at_request.yaw);
          const double s = std::sin(pose_at_request.yaw);

          glider::Pose p;
          p.x = pose_at_request.x + c * fwd - s * left;
          p.y = pose_at_request.y + s * fwd + c * left;
          p.z = pose_at_request.z - response->centre.position.z;

          processing_->setDetectedPooPose(p);
          RCLCPP_INFO(this->get_logger(),
            "Cube centre (world): (%.2f, %.2f, %.2f)", p.x, p.y, p.z);
        });
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Sonar service not ready yet");
    }
  }
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