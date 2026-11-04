#include "glider_processing.h"

#include <algorithm>

namespace glider
{

GliderProcessing::GliderProcessing(
  const Pose & home,
  const Pose & truck,
  double safe_height,
  double pickup_height,
  double align_height,
  double xy_tolerance,
  double z_tolerance)
: home_(home),
  truck_(truck),
  safe_height_(safe_height),
  pickup_height_(pickup_height),
  align_height_(align_height),
  xy_tolerance_(xy_tolerance),
  z_tolerance_(z_tolerance),
  current_pose_(home)
{
  status_msg_ = "STANDBY: waiting for poo locations";
}

bool GliderProcessing::isCarryingPoo() const
{
  return carrying_poo_;
}

std::string GliderProcessing::currentModelName() const
{
  if (current_goal_idx_ < goals_.size()) {
    return goals_[current_goal_idx_].model_name;
  }
  return "";
}

void GliderProcessing::setPose(const Pose & pose)
{
  current_pose_ = pose;
}

void GliderProcessing::addPooGoal(const Goal & goal)
{
  if (!goal.is_poo) {
    return;
  }

  goals_.push_back(goal);

  if (state_ == State::STANDBY) {
    current_goal_idx_ = 0;
    transition(State::TAKEOFF);
  }
}

void GliderProcessing::setPooGoals(const std::vector<Goal> & goals)
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

std::pair<VelocityCommand, std::string>
GliderProcessing::tick()
{
  command_ = VelocityCommand{};

  switch (state_) {
    case State::STANDBY:       handleStandby();       break;
    case State::TAKEOFF:       handleTakeoff();       break;
    case State::GOTO_POO:      handleGotoPoo();       break;
    case State::DESCEND:       handleDescend();       break;
    case State::ALIGN:         handleAlign();         break;
    case State::FINAL_DESCEND: handleFinalDescend();  break;
    case State::PICKUP:        handlePickup();        break;
    case State::ASCEND:        handleAscend();        break;
    case State::GOTO_TRUCK:    handleGoToTruck();     break;
    case State::DROP_OFF:      handleDropOff();       break;
    case State::RETURN_HOME:   handleReturnHome();    break;
  }

  return {command_, status_msg_};
}

// ---------------------------------------------------------------------------
// Reusable motion helpers
// ---------------------------------------------------------------------------

bool GliderProcessing::goToAltitude(double target_z)
{
  const double z_error = target_z - current_pose_.z;
  command_.vz = clamp(kp_z_ * z_error, -max_z_speed_, max_z_speed_);

  if (std::abs(z_error) <= z_tolerance_) {
    command_.vz = 0.0;
    return true;
  }
  return false;
}

bool GliderProcessing::goToXY(double x, double y)
{
  Pose target;
  target.x = x;
  target.y = y;
  target.z = current_pose_.z;

  command_ = positionControl(target);
  command_.vz = 0.0;

  return xyClose(current_pose_, target);
}

bool GliderProcessing::goToSafeHeight()
{
  return goToAltitude(safe_height_);
}

bool GliderProcessing::descendToHeight(double target_height, bool check_obstacles)
{
  // Hold / correct XY toward active target
  const Pose xy = activeTargetXY();
  command_.vx = clamp(kp_xy_ * (xy.x - current_pose_.x), -max_xy_speed_, max_xy_speed_);
  command_.vy = clamp(kp_xy_ * (xy.y - current_pose_.y), -max_xy_speed_, max_xy_speed_);

  if (!height_sensor_valid_) {
    // Odometry fallback
    double ref_z = current_pose_.z;
    if (current_goal_idx_ < goals_.size()) {
      ref_z = goals_[current_goal_idx_].pose.z + target_height;
    }
    const double z_error = ref_z - current_pose_.z;
    command_.vz = clamp(kp_z_ * z_error, -max_z_speed_, max_z_speed_);

    if (current_pose_.z < 0.35 && command_.vz < 0.0) {
      command_.vz = 0.0;
      command_ = worldToBody(command_);
      return true;
    }

    if (std::abs(z_error) <= z_tolerance_) {
      command_.vz = 0.0;
      command_ = worldToBody(command_);
      return true;
    }

    command_ = worldToBody(command_);
    return false;
  }

  // Obstacle gate: something closer than the height we want to stop at
  if (check_obstacles &&
      height_above_poo_ < target_height - z_tolerance_)
  {
    // Cycle ±X / ±Y in world frame
    static const double dir_x[4] = {+1.0, -1.0, 0.0, 0.0};
    static const double dir_y[4] = {0.0, 0.0, +1.0, -1.0};

    command_.vx = kSlideSpeed * dir_x[slide_dir_];
    command_.vy = kSlideSpeed * dir_y[slide_dir_];
    command_.vz = 0.0;

    ++slide_ticks_;
    if (slide_ticks_ >= kSlideTicksPerDir) {
      slide_ticks_ = 0;
      slide_dir_ = (slide_dir_ + 1) % 4;
    }

    status_msg_ = "DESCEND: obstacle below, sliding clear";
    command_ = worldToBody(command_);
    return false;
  }

  // Path is clear (or no obstacle check) → normal sensor descent
  slide_ticks_ = 0;  // reset slide state once clear

  const double height_error = target_height - height_above_poo_;
  command_.vz = clamp(kp_z_ * height_error, -max_z_speed_, max_z_speed_);

  if (current_pose_.z < 0.35 && command_.vz < 0.0) {
    command_.vz = 0.0;
    command_ = worldToBody(command_);
    return true;
  }

  if (std::abs(height_error) <= z_tolerance_) {
    command_.vz = 0.0;
    command_ = worldToBody(command_);
    return true;
  }

  command_ = worldToBody(command_);
  return false;
}

// ---------------------------------------------------------------------------
// State handlers
// ---------------------------------------------------------------------------

void GliderProcessing::handleStandby()
{
  command_ = VelocityCommand{};
  status_msg_ = "STANDBY: at research facility";
}

void GliderProcessing::handleTakeoff()
{
  status_msg_ = "TAKEOFF: climbing to safe height";
  if (goToSafeHeight()) {
    transition(State::GOTO_POO);
  }
}

void GliderProcessing::handleGotoPoo()
{
  if (current_goal_idx_ >= goals_.size()) {
    transition(State::RETURN_HOME);
    return;
  }

  const Goal & goal = goals_[current_goal_idx_];
  status_msg_ = "GOTO_POO: flying above poo location";

  if (goToXY(goal.pose.x, goal.pose.y)) {
    transition(State::DESCEND);
  }
}

void GliderProcessing::handleDescend()
{
  if (current_goal_idx_ >= goals_.size()) {
    transition(State::RETURN_HOME);
    return;
  }

  status_msg_ = "DESCEND: to align height";
  if (descendToHeight(align_height_, /*check_obstacles=*/true)) {
    transition(State::ALIGN);
  }
}

void GliderProcessing::handleAlign()
{
  if (current_goal_idx_ >= goals_.size()) {
    transition(State::RETURN_HOME);
    return;
  }

  command_.vz = 0.0;

  switch (align_phase_) {

    case AlignPhase::WAIT_DETECTION:
    case AlignPhase::WAIT_CHECK: {
      ++settle_ticks_;
      const bool checking = (align_phase_ == AlignPhase::WAIT_CHECK);

      if (cube_centre_requested_ &&
          settle_ticks_ > kSettleTicks + kRequestTimeoutTicks)
      {
        setCubeCentreUnavailable();
      }

      if (detection_failures_ >= kMaxFailures) {
        advanceAlignGridOrGiveUp();
        return;
      }

      if (!new_detection_) {
        status_msg_ = checking ? "ALIGN: verifying centre"
                               : "ALIGN: waiting for cube centre";
        return;
      }

      new_detection_ = false;
      const double dist = std::hypot(
        detected_poo_pose_.x - current_pose_.x,
        detected_poo_pose_.y - current_pose_.y);

      if (checking) {
        if (dist <= 1.5 * align_tolerance_) {
          status_msg_ = "ALIGN: centred, descending to pick up";
          transition(State::FINAL_DESCEND);
          return;
        }
        if (++align_attempts_ >= kMaxAttempts) {
          advanceAlignGridOrGiveUp();
          return;
        }
      }

      align_phase_ = AlignPhase::MOVE;
      status_msg_ = "ALIGN: moving over cube";
      return;
    }

    case AlignPhase::MOVE: {
      const double ex = detected_poo_pose_.x - current_pose_.x;
      const double ey = detected_poo_pose_.y - current_pose_.y;
      const double dist = std::hypot(ex, ey);

      if (dist <= align_tolerance_) {
        // Arrived at cell / cube estimate → settle and re-query sonar
        align_phase_ = AlignPhase::WAIT_CHECK;
        settle_ticks_ = 0;
        cube_centre_requested_ = false;
        new_detection_ = false;
        status_msg_ = "ALIGN: over target, settling";
        return;
      }

      const double speed = std::min(kp_xy_ * dist, kAlignMaxSpeed);
      command_.vx = speed * ex / dist;
      command_.vy = speed * ey / dist;
      status_msg_ = "ALIGN: moving over cube";
      break;
    }
  }

  command_ = worldToBody(command_);
}

void GliderProcessing::handleFinalDescend()
{
  if (current_goal_idx_ >= goals_.size()) {
    transition(State::RETURN_HOME);
    return;
  }

  status_msg_ = "FINAL DESCEND: sensor → pickup height";
  if (descendToHeight(pickup_height_, /*check_obstacles=*/false)) {
    transition(State::PICKUP);
  }
}

void GliderProcessing::handlePickup()
{
  command_ = VelocityCommand{};
  carrying_poo_ = true;
  clearDetectedPooPose();
  status_msg_ = "PICKUP: poo collected";
  transition(State::ASCEND);
}

void GliderProcessing::handleAscend()
{
  status_msg_ = "ASCEND: returning to safe height";
  if (goToSafeHeight()) {
    // Carrying → truck; not carrying (e.g. align gave up) → home
    if (carrying_poo_) {
      transition(State::GOTO_TRUCK);
    } else {
      transition(State::RETURN_HOME);
    }
  }
}

void GliderProcessing::handleGoToTruck()
{
  status_msg_ = "GOTO_TRUCK: returning to research facility";
  if (goToXY(truck_.x, truck_.y)) {
    transition(State::DROP_OFF);
  }
}

void GliderProcessing::handleDropOff()
{
  status_msg_ = "DROP_OFF: descending to truck";
  // No obstacle check; truck deck height from odometry
  if (goToAltitude(truck_.z)) {
    carrying_poo_ = false;
    current_goal_idx_++;
    transition(State::RETURN_HOME);
  }
}

void GliderProcessing::handleReturnHome()
{
  // More poos left → take off again for the next one
  if (current_goal_idx_ < goals_.size()) {
    transition(State::TAKEOFF);
    return;
  }

  status_msg_ = "RETURN_HOME: returning to research facility";

  if (!xyClose(current_pose_, home_)) {
    goToXY(home_.x, home_.y);
    return;
  }

  // XY at home → settle in Z
  if (goToAltitude(home_.z)) {
    goals_.clear();
    current_goal_idx_ = 0;
    transition(State::STANDBY);
  }
}

// ---------------------------------------------------------------------------
// Align grid search
// ---------------------------------------------------------------------------

void GliderProcessing::advanceAlignGridOrGiveUp()
{
  if (!align_grid_origin_set_) {
    align_grid_origin_.x = current_pose_.x;
    align_grid_origin_.y = current_pose_.y;
    align_grid_origin_set_ = true;
    align_grid_index_ = 0;
  }

  if (align_grid_index_ >= 8) {
    // Grid exhausted → climb then home (not carrying)
    status_msg_ = "ALIGN: grid exhausted, returning home";
    carrying_poo_ = false;
    clearDetectedPooPose();
    goals_.clear();
    current_goal_idx_ = 0;
    transition(State::ASCEND);
    return;
  }

  const double tx = align_grid_origin_.x + kGridOffsetX[align_grid_index_];
  const double ty = align_grid_origin_.y + kGridOffsetY[align_grid_index_];
  ++align_grid_index_;

  // Fly to next grid cell, then re-query sonar
  detected_poo_pose_.x = tx;
  detected_poo_pose_.y = ty;
  detected_poo_valid_ = true;
  new_detection_ = false;
  cube_centre_requested_ = false;
  detection_failures_ = 0;
  align_attempts_ = 0;
  settle_ticks_ = 0;
  align_phase_ = AlignPhase::MOVE;
  status_msg_ = "ALIGN: grid search next cell";
}

void GliderProcessing::resetAlign()
{
  align_phase_ = AlignPhase::WAIT_DETECTION;
  cube_centre_requested_ = false;
  new_detection_ = false;
  align_attempts_ = 0;
  detection_failures_ = 0;
  settle_ticks_ = 0;
  detected_poo_valid_ = false;
  align_grid_index_ = 0;
  align_grid_origin_set_ = false;
}

// ---------------------------------------------------------------------------
// Misc
// ---------------------------------------------------------------------------

void GliderProcessing::transition(State new_state)
{
  if (new_state == State::ALIGN) {
    resetAlign();
  }
  if (new_state == State::DESCEND) {
    slide_dir_ = 0;
    slide_ticks_ = 0;
  }
  state_ = new_state;
}

bool GliderProcessing::xyClose(const Pose & a, const Pose & b) const
{
  return std::hypot(a.x - b.x, a.y - b.y) <= xy_tolerance_;
}

double GliderProcessing::clamp(double value, double min, double max) const
{
  return std::max(min, std::min(value, max));
}

VelocityCommand GliderProcessing::positionControl(const Pose & target) const
{
  VelocityCommand command;

  const double x_error = target.x - current_pose_.x;
  const double y_error = target.y - current_pose_.y;

  command.vx = clamp(kp_xy_ * x_error, -max_xy_speed_, max_xy_speed_);
  command.vy = clamp(kp_xy_ * y_error, -max_xy_speed_, max_xy_speed_);

  return worldToBody(command);
}

VelocityCommand GliderProcessing::worldToBody(
  const VelocityCommand & world_command) const
{
  VelocityCommand body_command = world_command;

  const double c = std::cos(current_pose_.yaw);
  const double s = std::sin(current_pose_.yaw);

  body_command.vx =  c * world_command.vx + s * world_command.vy;
  body_command.vy = -s * world_command.vx + c * world_command.vy;

  return body_command;
}

Pose GliderProcessing::activeTargetXY() const
{
  Pose target;
  if (detected_poo_valid_) {
    target.x = detected_poo_pose_.x;
    target.y = detected_poo_pose_.y;
  } else if (current_goal_idx_ < goals_.size()) {
    target.x = goals_[current_goal_idx_].pose.x;
    target.y = goals_[current_goal_idx_].pose.y;
  } else {
    target.x = current_pose_.x;
    target.y = current_pose_.y;
  }
  return target;
}

void GliderProcessing::setDetectedPooPose(const Pose & pose)
{
  if (state_ != State::ALIGN) {
    return;
  }
  detected_poo_pose_ = pose;
  detected_poo_valid_ = true;
  new_detection_ = true;
  cube_centre_requested_ = false;
  detection_failures_ = 0;
}

void GliderProcessing::setCubeCentreUnavailable()
{
  cube_centre_requested_ = false;
  ++detection_failures_;
  settle_ticks_ = 0;
}

bool GliderProcessing::wantsCubeCentre() const
{
  return state_ == State::ALIGN &&
         align_phase_ != AlignPhase::MOVE &&
         !cube_centre_requested_ &&
         !new_detection_ &&
         settle_ticks_ >= kSettleTicks;
}

void GliderProcessing::clearDetectedPooPose()
{
  detected_poo_valid_ = false;
}

}  // namespace glider