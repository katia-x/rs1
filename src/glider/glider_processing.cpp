#include "glider_processing.h"

#include <algorithm>
#include <cmath>

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
  const Goal * goal = currentGoal();
  return goal != nullptr ? goal->model_name : "";
}

const Goal * GliderProcessing::currentGoal() const
{
  return current_goal_idx_ < goals_.size() ? &goals_[current_goal_idx_] : nullptr;
}

void GliderProcessing::setPose(const Pose & pose)
{
  current_pose_ = pose;
}


/*
 * Add a poo location to the mission.
 *
 * Mission will eventually be able to send multiple poo locations.
 */
void GliderProcessing::addPooGoal(const Goal & goal)
{
  if (!goal.is_poo) {
    return;
  }

  goals_.push_back(goal);

  // If we were waiting at the research facility, start the mission.
  if (state_ == State::STANDBY) {
    current_goal_idx_ = 0;
    transition(State::GOTO_POO);
  }
}


/*
 * Replace the current list of poo locations.
 */
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
    transition(State::GOTO_POO);
  }
}


/*
 * Sonar inputs. Each reading is only trusted for kSensorTimeoutTicks:
 * when the glider is out of the camera's range the sonar publishes nothing,
 * and an old reading must not be mistaken for the current one.
 */
void GliderProcessing::setHeightAbovePoo(double height)
{
  if (std::isfinite(height) && height >= 0.0) {
    height_above_poo_ = height;
    height_age_ticks_ = 0;
  }
}

void GliderProcessing::clearHeightAbovePoo()
{
  height_age_ticks_ = kStaleTicks;
}

void GliderProcessing::setNearestBelow(double range)
{
  if (std::isfinite(range) && range >= 0.0) {
    nearest_below_ = range;
    nearest_age_ticks_ = 0;
  }
}


/*
 * Main processing loop: the state machine for the whole mission.
 */
std::pair<VelocityCommand, std::string> GliderProcessing::tick()
{
  command_ = VelocityCommand{};

  if (height_age_ticks_ < kStaleTicks) {
    ++height_age_ticks_;
  }
  if (nearest_age_ticks_ < kStaleTicks) {
    ++nearest_age_ticks_;
  }

  switch (state_) {
    case State::STANDBY:       handleStandby();      break;
    case State::GOTO_POO:      handleGotoPoo();      break;
    case State::DESCEND:       handleDescend();      break;
    case State::ALIGN:         handleAlign();        break;
    case State::FINAL_DESCEND: handleFinalDescend(); break;
    case State::PICKUP:        handlePickup();       break;
    case State::GOTO_TRUCK:    handleGoToTruck();    break;
    case State::DROP_OFF:      handleDropOff();      break;
    case State::RETURN_HOME:   handleReturnHome();   break;
    case State::LAND:          handleLand();         break;
  }

  return {command_, status_msg_};
}


// ---------------------------------------------------------------------
// Reusable building blocks
// ---------------------------------------------------------------------

/*
 * Go to somewhere: climb straight up to safe height (z only), then fly to
 * the target in xy at that height. Returns true once over the target.
 * transition() resets the phase, so every new state starts with the climb.
 */
bool GliderProcessing::flyTo(const Pose & target, const std::string & label)
{
  if (fly_phase_ == FlyPhase::CLIMB) {
    const double z_error = safe_height_ - current_pose_.z;
    command_.vz = clamp(kp_z_ * z_error, -max_z_speed_, max_z_speed_);
    status_msg_ = label + ": climbing to safe height";

    if (std::abs(z_error) <= z_tolerance_) {
      command_.vz = 0.0;
      fly_phase_ = FlyPhase::TRANSIT;
    }
    return false;
  }

  moveXY(target);   // vz stays 0: hold safe height
  status_msg_ = label + ": flying to target";

  if (xyClose(current_pose_, target)) {
    command_.vx = 0.0;
    command_.vy = 0.0;
    return true;
  }
  return false;
}


/*
 * Go down to a height. The height is measured with the sonar when it has a
 * fresh reading and with odometry (absolute z) otherwise.
 *
 * check_obstacles: if something below the glider is taller than the altitude
 * we are descending to, stop and report BLOCKED (the caller decides what to
 * do about it). Pickup and drop-off pass false: the thing below is the target.
 */
GliderProcessing::DescendStatus GliderProcessing::descendTo(
  HeightRef ref, double height, double odom_z, bool check_obstacles, double min_z)
{
  if (check_obstacles && obstacleBelow(odom_z)) {
    command_.vz = 0.0;
    return DescendStatus::BLOCKED;
  }

  double error = odom_z - current_pose_.z;   // odometry fallback
  if (ref == HeightRef::GROUND && heightSensorFresh()) {
    error = height - height_above_poo_;
  } else if (ref == HeightRef::SURFACE && nearestBelowFresh()) {
    error = height - nearest_below_;
  }

  command_.vz = clamp(kp_z_ * error, -max_z_speed_, max_z_speed_);

  // Safety floor: counts as arrived
  if (current_pose_.z < min_z && command_.vz < 0.0) {
    command_.vz = 0.0;
    return DescendStatus::ARRIVED;
  }

  if (std::abs(error) <= z_tolerance_) {
    command_.vz = 0.0;
    return DescendStatus::ARRIVED;
  }

  return DescendStatus::DESCENDING;
}


/*
 * Move in xy toward a world-frame target.
 */
void GliderProcessing::moveXY(const Pose & target, double max_speed)
{
  const VelocityCommand v = positionControl(target, max_speed);
  command_.vx = v.vx;
  command_.vy = v.vy;
}


/*
 * Is something below us taller than the altitude we want to descend to?
 * Uses the nearest-surface sonar reading; with no fresh reading the sonar
 * sees nothing within its range, so there is nothing to avoid (yet).
 */
bool GliderProcessing::obstacleBelow(double target_z) const
{
  if (!nearestBelowFresh()) {
    return false;
  }
  const double top_z = current_pose_.z - kCameraDrop - nearest_below_;
  return top_z > target_z - kObstacleClearance;
}


/*
 * Slide sideways until nothing tall is below us. Slides along +x; if that is
 * still blocked after kSlideLegLength it turns 90 degrees and keeps going
 * (a square spiral). Returns false after kMaxSlideLegs legs.
 */
bool GliderProcessing::slideClear()
{
  if (!sliding_) {
    sliding_ = true;
    slide_legs_ = 0;
    slide_leg_start_ = current_pose_;
  }

  VelocityCommand world;
  world.vx = kSlideSpeed * slide_dir_x_;
  world.vy = kSlideSpeed * slide_dir_y_;
  const VelocityCommand body = worldToBody(world);
  command_.vx = body.vx;
  command_.vy = body.vy;

  // Wherever we are when the way down is clear becomes the descent column.
  column_xy_ = current_pose_;

  if (distXY(current_pose_, slide_leg_start_) > kSlideLegLength) {
    const double dx = slide_dir_x_;
    slide_dir_x_ = -slide_dir_y_;
    slide_dir_y_ = dx;
    slide_leg_start_ = current_pose_;
    if (++slide_legs_ >= kMaxSlideLegs) {
      return false;
    }
  }
  return true;
}


/*
 * Give up on the current poo: skip it and head home (or on to the next one).
 */
void GliderProcessing::abandonPoo(const std::string & reason)
{
  status_msg_ = reason + ", skipping this poo";
  clearDetectedPooPose();
  ++current_goal_idx_;
  transition(State::RETURN_HOME);
}


// ---------------------------------------------------------------------
// State handlers
// ---------------------------------------------------------------------

void GliderProcessing::handleStandby()
{
  status_msg_ = "STANDBY: at research facility";
}


void GliderProcessing::handleGotoPoo()
{
  const Goal * goal = currentGoal();
  if (goal == nullptr) {
    transition(State::RETURN_HOME);
    return;
  }

  if (flyTo(goal->pose, "GOTO_POO")) {
    transition(State::DESCEND);
  }
}


/*
 * DESCEND: from safe height down to the align height above the poo.
 * Checks for obstacles below; if something taller than the align height is
 * there, slide sideways until clear, then carry on down that column.
 */
void GliderProcessing::handleDescend()
{
  const Goal * goal = currentGoal();
  if (goal == nullptr) {
    transition(State::RETURN_HOME);
    return;
  }

  const DescendStatus result = descendTo(
    HeightRef::SURFACE, align_height_, goal->pose.z + align_height_, true);

  if (result == DescendStatus::BLOCKED) {
    status_msg_ = "DESCEND: obstacle below, sliding sideways";
    if (!slideClear()) {
      abandonPoo("DESCEND: no clear way down");
    }
    return;
  }

  sliding_ = false;
  moveXY(column_xy_);   // hold the (possibly shifted) descent column
  status_msg_ = "DESCEND: descending to align height";

  if (result == DescendStatus::ARRIVED) {
    if (distXY(current_pose_, goal->pose) > kMaxSlideFromGoal) {
      abandonPoo("DESCEND: poo is under an obstacle");
      return;
    }
    transition(State::ALIGN);
  }
}


/*
 * ALIGN: x/y only (vz stays 0, so waiting = hovering).
 *
 *  APPROACH        get back over the poo location
 *  WAIT_DETECTION  hover, ask the sonar for the cube centre
 *  MOVE            fly over the reported centre
 *  WAIT_CHECK      hover, ask again; close enough -> FINAL_DESCEND
 *  SEARCH_MOVE     cube not seen: visit the next point of the 3x3 grid
 *
 * If the whole grid fails (or centring keeps failing) the poo is skipped.
 */
void GliderProcessing::handleAlign()
{
  const Goal * goal = currentGoal();
  if (goal == nullptr) {
    transition(State::RETURN_HOME);
    return;
  }

  switch (align_phase_) {

    case AlignPhase::APPROACH: {
      status_msg_ = "ALIGN: moving over poo location";
      if (distXY(current_pose_, goal->pose) <= align_tolerance_) {
        align_origin_ = goal->pose;   // centre of the search grid
        startWaiting(AlignPhase::WAIT_DETECTION);
        return;
      }
      moveXY(goal->pose);
      return;
    }

    case AlignPhase::WAIT_DETECTION:
    case AlignPhase::WAIT_CHECK: {
      ++settle_ticks_;
      const bool checking = (align_phase_ == AlignPhase::WAIT_CHECK);
      status_msg_ = checking ? "ALIGN: verifying centre" : "ALIGN: looking for cube";

      // A request went out but no reply came back: count it as a failure.
      if (cube_centre_requested_ &&
        settle_ticks_ > kSettleTicks + kRequestTimeoutTicks)
      {
        setCubeCentreUnavailable();
      }

      // Cube not seen from here: try the next point of the grid.
      if (point_failures_ >= kRequestsPerPoint) {
        point_failures_ = 0;
        if (search_index_ >= kSearchPoints) {
          abandonPoo("ALIGN: cube not found");
          return;
        }
        ++search_index_;
        align_phase_ = AlignPhase::SEARCH_MOVE;
        status_msg_ = "ALIGN: cube not seen, trying next grid point";
        return;
      }

      if (!new_detection_) {
        return;
      }
      new_detection_ = false;

      if (checking) {
        const double dist = distXY(detected_poo_pose_, current_pose_);
        if (dist <= 1.5 * align_tolerance_) {
          status_msg_ = "ALIGN: centred, descending to pick up";
          transition(State::FINAL_DESCEND);
          return;
        }
        if (++align_attempts_ >= kMaxAttempts) {
          abandonPoo("ALIGN: could not centre");
          return;
        }
      }

      align_phase_ = AlignPhase::MOVE;
      return;
    }

    case AlignPhase::MOVE: {
      status_msg_ = "ALIGN: moving over cube";
      if (distXY(detected_poo_pose_, current_pose_) <= align_tolerance_) {
        startWaiting(AlignPhase::WAIT_CHECK);
        return;
      }
      moveXY(detected_poo_pose_, kAlignMaxSpeed);
      return;
    }

    case AlignPhase::SEARCH_MOVE: {
      status_msg_ = "ALIGN: searching around poo location";
      const Pose target = searchPoint(search_index_ - 1);
      if (distXY(current_pose_, target) <= align_tolerance_) {
        startWaiting(AlignPhase::WAIT_DETECTION);
        return;
      }
      moveXY(target, kAlignMaxSpeed);
      return;
    }
  }
}


/*
 * FINAL DESCEND: straight down onto the cube, holding xy over its centre.
 * No obstacle check: the cube is what we are going to.
 */
void GliderProcessing::handleFinalDescend()
{
  const Goal * goal = currentGoal();
  if (goal == nullptr) {
    transition(State::RETURN_HOME);
    return;
  }

  moveXY(activeTargetXY());
  status_msg_ = "FINAL DESCEND: descending to pickup height";

  const DescendStatus result = descendTo(
    HeightRef::GROUND, pickup_height_, goal->pose.z + pickup_height_,
    false, kMinPickupZ);

  if (result == DescendStatus::ARRIVED) {
    transition(State::PICKUP);
  }
}


/*
 * PICKUP: there is no physical gripper in RS1, so reaching the pickup
 * height counts as a successful pickup.
 */
void GliderProcessing::handlePickup()
{
  carrying_poo_ = true;
  clearDetectedPooPose();

  status_msg_ = "PICKUP: poo collected";

  transition(State::GOTO_TRUCK);
}


void GliderProcessing::handleGoToTruck()
{
  if (flyTo(truck_, "GOTO_TRUCK")) {
    transition(State::DROP_OFF);
  }
}


/*
 * DROP OFF: descend onto whatever is on the truck (bed, or cubes already
 * dropped on it), measured by the nearest-surface sonar reading.
 */
void GliderProcessing::handleDropOff()
{
  status_msg_ = "DROP_OFF: descending to truck bed";

  const DescendStatus result = descendTo(
    HeightRef::SURFACE, dropoff_height_, truck_.z, false);

  if (result == DescendStatus::ARRIVED) {
    carrying_poo_ = false;
    ++current_goal_idx_;
    transition(State::RETURN_HOME);
  }
}


void GliderProcessing::handleReturnHome()
{
  // More poo locations queued: carry on with the next one.
  if (current_goal_idx_ < goals_.size()) {
    transition(State::GOTO_POO);
    return;
  }

  if (flyTo(home_, "RETURN_HOME")) {
    transition(State::LAND);
  }
}


void GliderProcessing::handleLand()
{
  status_msg_ = "LAND: descending at the research facility";

  const DescendStatus result = descendTo(
    HeightRef::ODOMETRY, 0.0, home_.z, false);

  if (result == DescendStatus::ARRIVED) {
    goals_.clear();
    current_goal_idx_ = 0;
    transition(State::STANDBY);
  }
}


// ---------------------------------------------------------------------
// Transitions and ALIGN helpers
// ---------------------------------------------------------------------

void GliderProcessing::transition(State new_state)
{
  fly_phase_ = FlyPhase::CLIMB;   // every go-to starts with the climb

  if (new_state == State::DESCEND) {
    resetDescend();
  } else if (new_state == State::ALIGN) {
    resetAlign();
  }
  state_ = new_state;
}

void GliderProcessing::resetDescend()
{
  sliding_ = false;
  slide_legs_ = 0;
  slide_dir_x_ = 1.0;
  slide_dir_y_ = 0.0;

  const Goal * goal = currentGoal();
  column_xy_ = goal != nullptr ? goal->pose : current_pose_;
}

void GliderProcessing::resetAlign()
{
  align_phase_ = AlignPhase::APPROACH;
  cube_centre_requested_ = false;
  new_detection_ = false;
  point_failures_ = 0;
  align_attempts_ = 0;
  settle_ticks_ = 0;
  search_index_ = 0;
  detected_poo_valid_ = false;
}

void GliderProcessing::startWaiting(AlignPhase phase)
{
  align_phase_ = phase;
  settle_ticks_ = 0;
  cube_centre_requested_ = false;
  new_detection_ = false;
}

// Grid point `index` (0..7): offsets in kSearchOffsets, in steps from the centre.
Pose GliderProcessing::searchPoint(int index) const
{
  Pose p = align_origin_;
  p.x += kSearchOffsets[index][0] * kSearchStep;
  p.y += kSearchOffsets[index][1] * kSearchStep;
  return p;
}

bool GliderProcessing::wantsCubeCentre() const
{
  return state_ == State::ALIGN &&
         (align_phase_ == AlignPhase::WAIT_DETECTION ||
         align_phase_ == AlignPhase::WAIT_CHECK) &&
         !cube_centre_requested_ &&
         !new_detection_ &&
         settle_ticks_ >= kSettleTicks;
}

void GliderProcessing::setDetectedPooPose(const Pose & pose)
{
  if (state_ != State::ALIGN) {
    return;  // late reply after we've moved on
  }
  detected_poo_pose_ = pose;
  detected_poo_valid_ = true;
  new_detection_ = true;
  cube_centre_requested_ = false;
  point_failures_ = 0;
}

void GliderProcessing::setCubeCentreUnavailable()
{
  cube_centre_requested_ = false;
  ++point_failures_;
  settle_ticks_ = 0;   // wait again before retrying
}

void GliderProcessing::clearDetectedPooPose()
{
  detected_poo_valid_ = false;
}


// ---------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------

bool GliderProcessing::xyClose(const Pose & a, const Pose & b) const
{
  return distXY(a, b) <= xy_tolerance_;
}

double GliderProcessing::distXY(const Pose & a, const Pose & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double GliderProcessing::clamp(double value, double min, double max) const
{
  return std::max(min, std::min(value, max));
}


/*
 * Simple proportional position controller. Errors are calculated in world
 * coordinates and returned as a body-frame command.
 */
VelocityCommand GliderProcessing::positionControl(
  const Pose & target, double max_speed) const
{
  const double limit = (max_speed > 0.0) ? max_speed : max_xy_speed_;

  VelocityCommand command;
  command.vx = clamp(kp_xy_ * (target.x - current_pose_.x), -limit, limit);
  command.vy = clamp(kp_xy_ * (target.y - current_pose_.y), -limit, limit);

  return worldToBody(command);
}


/*
 * Convert world-frame XY velocity to body-frame XY velocity, so the
 * controller works regardless of the Parrot's yaw.
 */
VelocityCommand GliderProcessing::worldToBody(
  const VelocityCommand & world_command) const
{
  VelocityCommand body_command = world_command;

  const double c = std::cos(current_pose_.yaw);
  const double s = std::sin(current_pose_.yaw);

  body_command.vx = c * world_command.vx + s * world_command.vy;
  body_command.vy = -s * world_command.vx + c * world_command.vy;

  return body_command;
}


/*
 * Where to hold in xy: the detected cube centre if we have one,
 * otherwise the goal location.
 */
Pose GliderProcessing::activeTargetXY() const
{
  Pose target;
  if (detected_poo_valid_) {
    target.x = detected_poo_pose_.x;
    target.y = detected_poo_pose_.y;
  } else if (const Goal * goal = currentGoal()) {
    target.x = goal->pose.x;
    target.y = goal->pose.y;
  } else {
    target.x = current_pose_.x;
    target.y = current_pose_.y;
  }
  return target;
}

}  // namespace glider