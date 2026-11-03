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

  // Future downward sensor input
  void setHeightAbovePoo(double height);
  void clearHeightAbovePoo();

  // Poo pose
  void setDetectedPooPose(const Pose & pose);
  void clearDetectedPooPose();

  // Called at fixed rate
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
  void handleReturnHome();
  void handleDropOff();
  

  void transition(State new_state);

  bool xyClose(
    const Pose & a,
    const Pose & b) const;

  double clamp(
    double value,
    double min,
    double max) const;

  VelocityCommand positionControl(
    const Pose & target) const;

  VelocityCommand worldToBody(
    const VelocityCommand & world_command) const;

  Pose home_;
  Pose truck_;

  double safe_height_;
  double pickup_height_;
  double align_height_;
  double dropoff_height_;

  double xy_tolerance_;
  double z_tolerance_;

  // Controller limits
  double max_xy_speed_{1.0};
  double max_z_speed_{1.0};

  // Proportional controller gains
  double kp_xy_{0.8};
  double kp_z_{0.8};

  State state_{State::STANDBY};

  Pose current_pose_;

  std::vector<Goal> goals_;
  std::size_t current_goal_idx_{0};

  bool carrying_poo_{false};

  // Height sensor
  bool height_sensor_valid_{false};
  double height_above_poo_{0.0};


  bool detected_poo_valid_{false};
  Pose detected_poo_pose_;

  // poo helper
  Pose activeTargetXY() const;

  VelocityCommand command_;
  std::string status_msg_;
};

}  // namespace glider