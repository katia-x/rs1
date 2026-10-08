/*ToDO:
Read from /glider/sensor (through glider node) & compute where to send glider 
to line up with cube.

maybe can add a stopping point @ around a few metres above poo? and run /glider/sonar
as a Service instead of a topic. So not running & computing continuously?
*/

#include "glider_processing.h"

#include <algorithm>

namespace glider
{

GliderProcessing::GliderProcessing(
  const Pose & home,
  double safe_height,
  double pickup_height,
  double xy_tolerance,
  double z_tolerance)
: home_(home),
  safe_height_(safe_height),
  pickup_height_(pickup_height),
  dropoff_height_(home.z),
  xy_tolerance_(xy_tolerance),
  z_tolerance_(z_tolerance),
  current_pose_(home)
{
  status_msg_ = "STANDBY: waiting for poo locations";
}


void GliderProcessing::setPose(const Pose & pose)
{
  current_pose_ = pose;
}


/*
 * Add a poo location to the mission.
 *
 * This is important because Mission will eventually be able
 * to send multiple poo locations.
 */
void GliderProcessing::addPooGoal(const Goal & goal)
{
  if (!goal.is_poo) {
    return;
  }

  goals_.push_back(goal);

  // If we were waiting at the research facility,
  // start the mission.
  if (state_ == State::STANDBY) {
    current_goal_idx_ = 0;
    transition(State::TAKEOFF);
  }
}


/*
 * Replace the current list of poo locations.
 *
 * This can be useful later if Mission sends the whole
 * mission list at once.
 */
void GliderProcessing::setPooGoals(
  const std::vector<Goal> & goals)
{
  goals_.clear();

  for (const auto & goal : goals) {
    if (goal.is_poo) {
      goals_.push_back(goal);
    }
  }

  current_goal_idx_ = 0;

  if (!goals_.empty() && state_ == State::STANDBY) {
    transition(State::TAKEOFF);
  }
}


/*
 * Receive the distance measured by a future downward sensor.
 */
void GliderProcessing::setHeightAbovePoo(double height)
{
  if (std::isfinite(height) && height >= 0.0) {
    height_above_poo_ = height;
    height_sensor_valid_ = true;
  }
}


void GliderProcessing::clearHeightAbovePoo()
{
  height_sensor_valid_ = false;
}


/*
 * Main processing loop.
 *
 * This contains the state machine for the entire Glider mission.
 */
std::pair<VelocityCommand, std::string>
GliderProcessing::tick()
{
  command_ = VelocityCommand{};

  switch (state_) {

    case State::STANDBY:
      handleStandby();
      break;

    case State::TAKEOFF:
      handleTakeoff();
      break;

    case State::GOTO_POO:
      handleGotoPoo();
      break;

    case State::DESCEND:
      handleDescend();
      break;

    case State::PICKUP:
      handlePickup();
      break;

    case State::ASCEND:
      handleAscend();
      break;

    case State::RETURN_HOME:
      handleReturnHome();
      break;

    case State::DROP_OFF:
      handleDropOff();
      break;
  }

  return {command_, status_msg_};
}


/*
 * ---------------------------------------------------------
 * STANDBY
 * ---------------------------------------------------------
 *
 * Glider remains at the research facility.
 */
void GliderProcessing::handleStandby()
{
  command_ = VelocityCommand{};

  status_msg_ =
    "STANDBY: at research facility";
}


/*
 * ---------------------------------------------------------
 * TAKEOFF
 * ---------------------------------------------------------
 *
 * Fly vertically to the safe height above the trees.
 */
void GliderProcessing::handleTakeoff()
{
  const double z_error =
    safe_height_ - current_pose_.z;

  command_.vz =
    clamp(
      kp_z_ * z_error,
      -max_z_speed_,
      max_z_speed_);

  status_msg_ =
    "TAKEOFF: climbing to safe height";

  if (std::abs(z_error) <= z_tolerance_) {

    command_.vz = 0.0;

    transition(State::GOTO_POO);
  }
}


/*
 * ---------------------------------------------------------
 * GOTO POO
 * ---------------------------------------------------------
 *
 * Fly horizontally to the poo location while remaining
 * at the safe height.
 */
void GliderProcessing::handleGotoPoo()
{
  if (current_goal_idx_ >= goals_.size()) {

    transition(State::RETURN_HOME);

    return;
  }

  const Goal & goal =
    goals_[current_goal_idx_];

  Pose target = goal.pose;

  // Stay at safe height.
  target.z = safe_height_;

  command_ =
    positionControl(target);

  command_.vz = 0.0;

  status_msg_ =
    "GOTO_POO: flying above poo location";

  if (xyClose(current_pose_, target)) {

    command_.vx = 0.0;
    command_.vy = 0.0;

    transition(State::DESCEND);
  }
}


/*
 * ---------------------------------------------------------
 * DESCEND
 * ---------------------------------------------------------
 *
 * Move straight down while continuously correcting XY
 * position.
 *
 * If a height sensor is available:
 *
 *     use sensor distance
 *
 * Otherwise:
 *
 *     temporarily use odometry
 *
 * The sensor version is what should eventually be used
 * for the real implementation.
 */
void GliderProcessing::handleDescend()
{
  if (current_goal_idx_ >= goals_.size()) {

    transition(State::RETURN_HOME);

    return;
  }

  const Goal & goal =
    goals_[current_goal_idx_];


  // -------------------------------------------------------
  // XY correction
  // -------------------------------------------------------

  const double x_error =
    goal.pose.x - current_pose_.x;

  const double y_error =
    goal.pose.y - current_pose_.y;

  command_.vx =
    clamp(
      kp_xy_ * x_error,
      -max_xy_speed_,
      max_xy_speed_);

  command_.vy =
    clamp(
      kp_xy_ * y_error,
      -max_xy_speed_,
      max_xy_speed_);


  // -------------------------------------------------------
  // Height control
  // -------------------------------------------------------

  if (height_sensor_valid_) {

    /*
     * Sensor measures the distance between the Glider
     * and the poo/ground.
     */
    const double height_error =
      height_above_poo_ - pickup_height_;

    /*
     * Positive vz = climb
     * Negative vz = descend
     */
    command_.vz =
      clamp(
        kp_z_ * height_error,
        -max_z_speed_,
        max_z_speed_);

    status_msg_ =
      "DESCEND: using height sensor";

    if (std::abs(height_error) <= z_tolerance_) {

      command_.vz = 0.0;

      transition(State::PICKUP);
    }

  } else {

    /*
     * TEMPORARY TEST FALLBACK.
     *
     * The current Parrot model does not provide a
     * downward Range sensor.
     *
     * Therefore use the known poo Z coordinate and
     * odometry for testing.
     */
    const double target_z =
      goal.pose.z + pickup_height_;

    const double height_error =
      target_z - current_pose_.z;

    command_.vz =
      clamp(
        kp_z_ * height_error,
        -max_z_speed_,
        max_z_speed_);

    status_msg_ =
      "DESCEND: odometry fallback (no height sensor)";

    if (std::abs(height_error) <= z_tolerance_) {

      command_.vz = 0.0;

      transition(State::PICKUP);
    }
  }


  /*
   * Convert the horizontal command from world coordinates
   * into the Parrot body frame because cmd_vel uses:
   *
   * linear.x = forward/backward
   * linear.y = left/right
   */
  command_ =
    worldToBody(command_);
}


/*
 * ---------------------------------------------------------
 * PICKUP
 * ---------------------------------------------------------
 *
 * There is currently no physical gripper in RS1.
 *
 * Therefore reaching the pickup height is treated as a
 * successful pickup for now.
 */
void GliderProcessing::handlePickup()
{
  command_ = VelocityCommand{};

  carrying_poo_ = true;

  status_msg_ =
    "PICKUP: poo collected";

  transition(State::ASCEND);
}


/*
 * ---------------------------------------------------------
 * ASCEND
 * ---------------------------------------------------------
 *
 * Return vertically to the safe height.
 */
void GliderProcessing::handleAscend()
{
  const double z_error =
    safe_height_ - current_pose_.z;

  command_.vz =
    clamp(
      kp_z_ * z_error,
      -max_z_speed_,
      max_z_speed_);

  status_msg_ =
    "ASCEND: returning to safe height";

  if (std::abs(z_error) <= z_tolerance_) {

    command_.vz = 0.0;

    transition(State::RETURN_HOME);
  }
}


/*
 * ---------------------------------------------------------
 * RETURN HOME
 * ---------------------------------------------------------
 *
 * Fly horizontally back to the research facility.
 */
void GliderProcessing::handleReturnHome()
{
  Pose target = home_;

  target.z = safe_height_;

  command_ =
    positionControl(target);

  command_.vz = 0.0;

  status_msg_ =
    "RETURN_HOME: returning to research facility";

  if (xyClose(current_pose_, target)) {

    command_.vx = 0.0;
    command_.vy = 0.0;

    transition(State::DROP_OFF);
  }
}


/*
 * ---------------------------------------------------------
 * DROP OFF
 * ---------------------------------------------------------
 *
 * Descend to the research facility drop-off height.
 */
void GliderProcessing::handleDropOff()
{
  const double z_error =
    dropoff_height_ - current_pose_.z;

  command_.vz =
    clamp(
      kp_z_ * z_error,
      -max_z_speed_,
      max_z_speed_);

  status_msg_ =
    "DROP_OFF: landing at research facility";


  if (std::abs(z_error) <= z_tolerance_) {

    command_ = VelocityCommand{};

    carrying_poo_ = false;

    ++current_goal_idx_;


    /*
     * More poo locations remain.
     */
    if (current_goal_idx_ < goals_.size()) {

      transition(State::TAKEOFF);
    }

    /*
     * No more poo locations.
     */
    else {

      goals_.clear();

      current_goal_idx_ = 0;

      transition(State::STANDBY);
    }
  }
}


/*
 * Change state.
 */
void GliderProcessing::transition(State new_state)
{
  state_ = new_state;
}


/*
 * Check whether two positions are sufficiently close
 * horizontally.
 */
bool GliderProcessing::xyClose(
  const Pose & a,
  const Pose & b) const
{
  return std::hypot(
    a.x - b.x,
    a.y - b.y) <= xy_tolerance_;
}


/*
 * Limit a value to a specified range.
 */
double GliderProcessing::clamp(
  double value,
  double min,
  double max) const
{
  return std::max(
    min,
    std::min(value, max));
}


/*
 * Simple proportional position controller.
 *
 * The errors are initially calculated in world coordinates.
 */
VelocityCommand GliderProcessing::positionControl(
  const Pose & target) const
{
  VelocityCommand command;

  const double x_error =
    target.x - current_pose_.x;

  const double y_error =
    target.y - current_pose_.y;

  command.vx =
    clamp(
      kp_xy_ * x_error,
      -max_xy_speed_,
      max_xy_speed_);

  command.vy =
    clamp(
      kp_xy_ * y_error,
      -max_xy_speed_,
      max_xy_speed_);

  return worldToBody(command);
}


/*
 * Convert world-frame XY velocity to body-frame XY velocity.
 *
 * This is the important part that allows the controller
 * to work regardless of the Parrot's yaw.
 */
VelocityCommand GliderProcessing::worldToBody(
  const VelocityCommand & world_command) const
{
  VelocityCommand body_command =
    world_command;

  const double c =
    std::cos(current_pose_.yaw);

  const double s =
    std::sin(current_pose_.yaw);

  body_command.vx =
    c * world_command.vx +
    s * world_command.vy;

  body_command.vy =
    -s * world_command.vx +
    c * world_command.vy;

  return body_command;
}

}  // namespace glider