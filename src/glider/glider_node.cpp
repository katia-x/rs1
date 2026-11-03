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

  this->declare_parameter("home_x", -17.5);
  this->declare_parameter("home_y", 13.0);
  this->declare_parameter("home_z", 0.2);

  this->declare_parameter("truck_x", -16.3);
  this->declare_parameter("truck_y", 16.5);
  this->declare_parameter("truck_z", 1.0);

  this->declare_parameter("safe_height", 15.0);
  this->declare_parameter("pickup_height", 0.2);
  this->declare_parameter("align_height", 1.5);

  this->declare_parameter("xy_tolerance", 0.4);
  this->declare_parameter("z_tolerance", 0.15);

  this->declare_parameter("control_rate", 20.0);


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

  goal.model_name = "poo_2";


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

// ---------------------------------------------------------
// Spawn a poo model at the truck
// ---------------------------------------------------------
void GliderNode::spawnPooAtTruck(const std::string & name)
{
  // Path to the same SDF we already use
  std::string sdf_path =
    ament_index_cpp::get_package_share_directory("rs1") +
    "/worlds/poo_cube/poo_cube.sdf";

  // Truck pose 
  double x = this->get_parameter("truck_x").as_double();
  double y = this->get_parameter("truck_y").as_double();
  double z = this->get_parameter("truck_z").as_double() - 0.2;  // slightly below glider

  std::stringstream ss;
  ss << "ros2 run ros_gz_sim create "
     << "-world rs1_terrain "
     << "-name " << name << " "
     << "-x " << x << " -y " << y << " -z " << z << " "
     << "-file " << sdf_path;

  int ret = std::system(ss.str().c_str());
  if (ret != 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to spawn model '%s' at truck", name.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Spawned model '%s' at truck", name.c_str());
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
      spawnPooAtTruck(current_model_name_);
      current_model_name_.clear();
    }
  }

  was_carrying = is_carrying;
  // call sonar service once when we enter ALIGN 
  static glider::State last_state = glider::State::STANDBY;
  const auto current_state = processing_->state();

  // if (current_state == glider::State::ALIGN && last_state != glider::State::ALIGN)
  // {
  //   if (sonar_client_->service_is_ready())
  //   {
  //     auto request = std::make_shared<rs1::srv::GetCubeCentre::Request>();

  //     sonar_client_->async_send_request(
  //       request,
  //       [this](rclcpp::Client<rs1::srv::GetCubeCentre>::SharedFuture future)
  //       {
  //         auto response = future.get();
  //         if (response->success)
  //         {
  //           glider::Pose p;
  //           p.x = response->centre.position.x;
  //           p.y = response->centre.position.y;
  //           p.z = response->centre.position.z;

  //           processing_->setDetectedPooPose(p);

  //           RCLCPP_INFO(this->get_logger(),
  //             "Sonar centre received: (%.2f, %.2f, %.2f)",
  //             p.x, p.y, p.z);
  //         }
  //         else
  //         {
  //           RCLCPP_WARN(this->get_logger(),
  //             "Sonar service failed: %s", response->message.c_str());
  //         }
  //       });
  //   }
  //   else
  //   {
  //     RCLCPP_WARN(this->get_logger(), "Sonar service not ready yet");
  //   }
  //}

  last_state = current_state;

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