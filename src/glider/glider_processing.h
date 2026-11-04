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
  TAKEOFF,
  GOTO_POO,
  DESCEND,
  ALIGN,
  FINAL_DESCEND,
  PICKUP,
  ASCEND,
  GOTO_TRUCK,
  DROP_OFF,
  RETURN_HOME
};

class GliderProcessing
{
public:
  GliderProcessing(
    const Pose & home,
    const Pose & truck,
    double safe_height = 15.0,
    double pickup_height = 0.40,
    double align_height = 1.5,
    double xy_tolerance = 0.4,
    double z_tolerance = 0.15);

  bool isCarryingPoo() const;
  std::string currentModelName() const;

  void setPose(const Pose & pose);

  void addPooGoal(const Goal & goal);
  void setPooGoals(const std::vector<Goal> & goals);

  void setHeightAbovePoo(double height);
  void clearHeightAbovePoo();

  void setDetectedPooPose(const Pose & pose);
  void clearDetectedPooPose();

  bool wantsCubeCentre() const;
  void markCubeCentreRequested() { cube_centre_requested_ = true; }
  void setCubeCentreUnavailable();
  void setAlignTolerance(double tol) { align_tolerance_ = tol; }

  std::pair<VelocityCommand, std::string> tick();

  State state() const { return state_; }
  bool carryingPoo() const { return carrying_poo_; }

private:
  void handleStandby();
  void handleTakeoff();
  void handleGotoPoo();
  void handleDescend();
  void handleAlign();
  void handleFinalDescend();
  void handlePickup();
  void handleAscend();
  void handleGoToTruck();
  void handleDropOff();
  void handleReturnHome();

  void transition(State new_state);

  // ---- Reusable motion helpers ----
  bool goToAltitude(double target_z);
  bool goToXY(double x, double y);
  bool goToSafeHeight();
  bool descendToHeight(double target_height, bool check_obstacles);

  bool xyClose(const Pose & a, const Pose & b) const;
  double clamp(double value, double min, double max) const;
  VelocityCommand positionControl(const Pose & target) const;
  VelocityCommand worldToBody(const VelocityCommand & world_command) const;
  Pose activeTargetXY() const;

  // Align helpers
  void resetAlign();
  void advanceAlignGridOrGiveUp();

  Pose home_;
  Pose truck_;

  double safe_height_;
  double pickup_height_;
  double align_height_;
  double xy_tolerance_;
  double z_tolerance_;

  double max_xy_speed_{1.0};
  double max_z_speed_{1.0};
  double kp_xy_{0.8};
  double kp_z_{0.8};

  State state_{State::STANDBY};
  Pose current_pose_;

  std::vector<Goal> goals_;
  std::size_t current_goal_idx_{0};
  bool carrying_poo_{false};

  bool height_sensor_valid_{false};
  double height_above_poo_{0.0};

  bool detected_poo_valid_{false};
  Pose detected_poo_pose_;

  // Align
  enum class AlignPhase { WAIT_DETECTION, MOVE, WAIT_CHECK };
  AlignPhase align_phase_{AlignPhase::WAIT_DETECTION};
  bool cube_centre_requested_{false};
  bool new_detection_{false};
  int align_attempts_{0};
  int detection_failures_{0};
  int settle_ticks_{0};
  double align_tolerance_{0.08};

  // 3x3 grid search (centre + 8 neighbours)
  int align_grid_index_{0};
  Pose align_grid_origin_{};
  bool align_grid_origin_set_{false};

  static constexpr int kSettleTicks = 20;
  static constexpr int kRequestTimeoutTicks = 60;
  static constexpr int kMaxAttempts = 4;
  static constexpr int kMaxFailures = 5;
  static constexpr double kAlignMaxSpeed = 0.4;
  static constexpr double kGridStep = 0.2;
  // Order: +x, +x-y, -x, -x, +y, +y, +x, +x  → 8 outer cells of 3x3
  static constexpr double kGridOffsetX[8] = {
    +0.2, +0.2,  0.0, -0.2, -0.2, -0.2,  0.0, +0.2};
  static constexpr double kGridOffsetY[8] = {
     0.0, -0.2, -0.2, -0.2,  0.0, +0.2, +0.2, +0.2};

  // Obstacle slide during first descend (±X / ±Y)
  int slide_dir_{0};      // 0=+X, 1=-X, 2=+Y, 3=-Y (world)
  int slide_ticks_{0};
  static constexpr int kSlideTicksPerDir = 40;  // ~2 s at 20 Hz
  static constexpr double kSlideSpeed = 0.25;

  VelocityCommand command_;
  std::string status_msg_;
};

}  // namespace glider