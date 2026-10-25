#ifndef HUNTER_TRAJECTORY__TRAJECTORY_SERVER_HPP_
#define HUNTER_TRAJECTORY__TRAJECTORY_SERVER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hunter_msgs/msg/trajectory.hpp"
#include "hunter_msgs/msg/waypoint.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hunter_trajectory
{

// One fully-populated trajectory point. parseCenterline()/parseRaceline()
// are each responsible for filling every field (synthesizing s_m/psi_rad
// where the source format doesn't provide them) so downstream logic never
// needs to know which format the data came from.
struct RawPoint
{
  double x_m = 0.0;
  double y_m = 0.0;
  double psi_rad = 0.0;
  double kappa_radpm = 0.0;  // synthesized (dpsi/ds) for centerline input
  double s_m = 0.0;
  double vx_mps = 0.0;       // overwritten by applySpeedProfile()
  double ax_mps2 = 0.0;      // informational; 0 if not available
};

enum class TrackFormat
{
  kCenterline,  // x_m, y_m, w_tr_right_m, w_tr_left_m -- no heading/curvature/speed
  kRaceline,    // s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
};

class TrajectoryServer : public rclcpp::Node
{
public:
  explicit TrajectoryServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  TrackFormat detectFormat(const std::string & path, const std::string & requested) const;

  // Each parse function synthesizes s_m (cumulative chord length), psi_rad
  // (central-difference heading), and kappa_radpm (central-difference
  // dpsi/ds) when the source format lacks them.
  std::vector<RawPoint> parseCenterline(const std::string & path) const;
  std::vector<RawPoint> parseRaceline(const std::string & path) const;

  // Drops a trailing point that coincides with the first point (within
  // close_loop_gap_threshold_m), returns {is_closed_loop, total_length_m}.
  std::pair<bool, double> dedupeClosingPoint(
    std::vector<RawPoint> & points, double gap_threshold_m) const;

  // Overwrites vx_mps on every point per the effective speed_source.
  void applySpeedProfile(
    std::vector<RawPoint> & points, const std::string & effective_speed_source,
    double min_speed_mps, double max_speed_mps, double lateral_accel_limit_mps2,
    double constant_speed_mps) const;

  // Rigidly transforms every point from the CSV's world frame into the odom
  // frame, given the world pose at which the odom frame originated (the robot
  // pose at controller activation, i.e. the spawn pose). kappa/s/speeds are
  // rotation-invariant and left untouched.
  void transformToOdomFrame(
    std::vector<RawPoint> & points, double origin_x_m, double origin_y_m,
    double origin_yaw_rad) const;

  hunter_msgs::msg::Trajectory buildTrajectoryMsg(
    const std::vector<RawPoint> & points, bool is_closed_loop, double total_length_m,
    const std::string & frame_id) const;
  nav_msgs::msg::Path buildPathMsg(const hunter_msgs::msg::Trajectory & traj) const;

  rclcpp::Publisher<hunter_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace hunter_trajectory

#endif  // HUNTER_TRAJECTORY__TRAJECTORY_SERVER_HPP_
