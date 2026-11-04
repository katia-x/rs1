#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace glider
{

struct Pose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct Goal
{
  Pose pose;
  bool is_poo{true};
  std::string model_name;
};

struct VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double yaw_rate{0.0};
};

enum class State
{
  STANDBY,
  GOTO_POO,        // climb to safe height, then fly over the poo
  DESCEND,         // descend to align height (checks for obstacles below)
  ALIGN,           // centre over the cube using the sonar service
  FINAL_DESCEND,   // descend to pickup height
  PICKUP,
  GOTO_TRUCK,      // climb to safe height, then fly over the truck
  DROP_OFF,        // descend onto the truck and release
  RETURN_HOME,     // climb to safe height, then fly home
  LAND
};

class GliderProcessing
{
public:
  GliderProcessing(
    const Pose & home,
    const Pose & truck,
    double safe_height = 15.0,
    double pickup_height = 0.5,
    double align_height = 1.5,
    double xy_tolerance = 0.4,
    double z_tolerance = 0.15);

  // For getting and destroying poos
  bool isCarryingPoo() const;
  std::string currentModelName() const;

  // Current Parrot position
  void setPose(const Pose & pose);

  // Add one poo location to the mission queue
  void addPooGoal(const Goal & goal);

  // Replace the whole poo mission queue
  void setPooGoals(const std::vector<Goal> & goals);

  // Sonar inputs (readings older than ~1.5 s are ignored)
  void setHeightAbovePoo(double height);   // distance to the ground below (floor)
  void clearHeightAbovePoo();
  void setNearestBelow(double range);      // distance to the nearest thing directly below

  // Cube centre handshake (the node calls the sonar service on our behalf)
  bool wantsCubeCentre() const;
  void markCubeCentreRequested() { cube_centre_requested_ = true; }
  void setDetectedPooPose(const Pose & pose);
  void setCubeCentreUnavailable();
  void clearDetectedPooPose();

  // Tunables
  void setAlignTolerance(double tol) { align_tolerance_ = tol; }
  void setDropoffHeight(double height) { dropoff_height_ = height; }

  // Called at fixed rate
  std::pair<VelocityCommand, std::string> tick();

  State state() const { return state_; }
  bool carryingPoo() const { return carrying_poo_; }

private:
  // -------------------------------------------------------------------
  // Tuning constants. Tick-based timings assume the 20 Hz control loop.
  // -------------------------------------------------------------------
  static constexpr int kStaleTicks = 1000;
  static constexpr int kSensorTimeoutTicks = 30;    // ~1.5 s
  static constexpr int kSettleTicks = 20;           // hover ~1 s before asking the sonar
  static constexpr int kRequestTimeoutTicks = 60;   // ~3 s with no service reply
  static constexpr int kRequestsPerPoint = 2;       // failed requests before moving on
  static constexpr int kMaxAttempts = 4;            // re-centre attempts after a failed check

  // ALIGN grid search: 3x3 points around the poo, `kSearchStep` apart.
  // Offsets (in steps, world frame) from the centre, in visiting order.
  static constexpr double kAlignMaxSpeed = 0.4;     // m/s
  static constexpr double kSearchStep = 0.4;        // m
  static constexpr int kSearchPoints = 8;
  static constexpr int kSearchOffsets[kSearchPoints][2] = {
    {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}};

  // Obstacle check while descending
  static constexpr double kCameraDrop = 0.05;       // camera hangs this far below base_link
  static constexpr double kObstacleClearance = 0.5; // keep this far above anything below
  static constexpr double kSlideSpeed = 0.5;        // m/s
  static constexpr double kSlideLegLength = 3.0;    // m before turning 90 degrees
  static constexpr int kMaxSlideLegs = 4;
  static constexpr double kMaxSlideFromGoal = 3.0;  // give up if the clear column is further than this

  static constexpr double kMinPickupZ = 0.35;       // never descend below this for pickup
  static constexpr double kNoFloor = -1.0e9;

  // -------------------------------------------------------------------
  // Types
  // -------------------------------------------------------------------
  enum class FlyPhase { CLIMB, TRANSIT };
  enum class DescendStatus { DESCENDING, ARRIVED, BLOCKED };
  // What a descent measures its height against
  enum class HeightRef
  {
    GROUND,     // sonar floor (distance to the ground)
    SURFACE,    // sonar nearest surface directly below (e.g. truck bed, cubes on it)
    ODOMETRY    // no sonar: absolute z only
  };
  enum class AlignPhase { APPROACH, WAIT_DETECTION, MOVE, WAIT_CHECK, SEARCH_MOVE };

  // -------------------------------------------------------------------
  // State handlers
  // -------------------------------------------------------------------
  void handleStandby();
  void handleGotoPoo();
  void handleDescend();
  void handleAlign();
  void handleFinalDescend();
  void handlePickup();
  void handleGoToTruck();
  void handleDropOff();
  void handleReturnHome();
  void handleLand();

  void transition(State new_state);

  // -------------------------------------------------------------------
  // Reusable building blocks
  // -------------------------------------------------------------------

  // Climb straight up to safe height (z only), then fly to `target` in xy.
  // Returns true once over the target. Status text is prefixed with `label`.
  bool flyTo(const Pose & target, const std::string & label);

  // Descend (z only) to `height` above `ref`, falling back to the absolute
  // z `odom_z` when the sonar has no fresh reading. With `check_obstacles`
  // it stops and reports BLOCKED if something below is taller than `odom_z`.
  // Never descends below `min_z`.
  DescendStatus descendTo(
    HeightRef ref, double height, double odom_z,
    bool check_obstacles, double min_z = kNoFloor);

  // Move in xy toward `target` (world frame in, body-frame command out).
  void moveXY(const Pose & target, double max_speed = -1.0);

  bool obstacleBelow(double target_z) const;
  bool slideClear();                                   // false once it gives up
  void abandonPoo(const std::string & reason);

  bool heightSensorFresh() const { return height_age_ticks_ <= kSensorTimeoutTicks; }
  bool nearestBelowFresh() const { return nearest_age_ticks_ <= kSensorTimeoutTicks; }
  const Goal * currentGoal() const;

  void resetDescend();
  void resetAlign();
  void startWaiting(AlignPhase phase);
  Pose searchPoint(int index) const;
  Pose activeTargetXY() const;

  bool xyClose(const Pose & a, const Pose & b) const;
  static double distXY(const Pose & a, const Pose & b);
  double clamp(double value, double min, double max) const;

  VelocityCommand positionControl(const Pose & target, double max_speed = -1.0) const;
  VelocityCommand worldToBody(const VelocityCommand & world_command) const;

  // -------------------------------------------------------------------
  // Data
  // -------------------------------------------------------------------
  Pose home_;
  Pose truck_;

  double safe_height_;
  double pickup_height_;
  double align_height_;
  double dropoff_height_{0.5};   // release height above the nearest surface below

  double xy_tolerance_;
  double z_tolerance_;
  double align_tolerance_{0.08};

  // Controller limits and gains
  double max_xy_speed_{1.0};
  double max_z_speed_{1.0};
  double kp_xy_{0.8};
  double kp_z_{0.8};

  State state_{State::STANDBY};
  Pose current_pose_;

  std::vector<Goal> goals_;
  std::size_t current_goal_idx_{0};
  bool carrying_poo_{false};

  // Sonar
  int height_age_ticks_{kStaleTicks};
  double height_above_poo_{0.0};
  int nearest_age_ticks_{kStaleTicks};
  double nearest_below_{0.0};

  // flyTo sub-machine
  FlyPhase fly_phase_{FlyPhase::CLIMB};

  // DESCEND: current descent column and sideways search
  Pose column_xy_;
  bool sliding_{false};
  int slide_legs_{0};
  Pose slide_leg_start_;
  double slide_dir_x_{1.0};
  double slide_dir_y_{0.0};

  // ALIGN
  AlignPhase align_phase_{AlignPhase::APPROACH};
  bool cube_centre_requested_{false};
  bool new_detection_{false};
  int point_failures_{0};
  int align_attempts_{0};
  int settle_ticks_{0};
  int search_index_{0};          // grid points started so far
  Pose align_origin_;            // centre of the 3x3 grid
  bool detected_poo_valid_{false};
  Pose detected_poo_pose_;

  VelocityCommand command_;
  std::string status_msg_;
};

}  // namespace glider