#ifndef HUNTER_CONTROL__HUNTER_CONTROL_COMMON_HPP_
#define HUNTER_CONTROL__HUNTER_CONTROL_COMMON_HPP_

#include <cstddef>
#include <vector>

#include "hunter_msgs/msg/waypoint.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Pure-geometry helpers shared by every tracker node (pid_tracker now;
// Phase 4-9's pure_pursuit/stanley/lqr/mpc/rl trackers later). Deliberately
// contains zero PID- or algorithm-specific logic -- only nearest-point
// search and cross-track/heading error, so every tracker computes
// TrackingDiagnostics identically and benchmark comparisons stay apples-to-apples.
namespace hunter_control_common
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double speed = 0.0;  // forward speed in the vehicle frame (odom.twist.twist.linear.x)
};

// nav_msgs/Odometry has no odom->base_link TF in this project
// (ackermann_like_controller.yaml sets enable_odom_tf:false), so this is the
// *only* correct way to get the robot's pose -- never use tf2_ros::Buffer
// for this.
Pose2D poseFromOdometry(const nav_msgs::msg::Odometry & odom);

double wrapToPi(double angle_rad);

struct TrackingErrorResult
{
  std::size_t index = 0;
  double lateral_error_m = 0.0;    // signed; +ve = robot left of path tangent
  double heading_error_rad = 0.0;  // signed, wrapped to [-pi, pi]
  double path_s_m = 0.0;
};

// Finds the nearest waypoint to `pose` and returns the signed cross-track
// and heading error against that waypoint's heading.
//
// Searches a window of `search_window` indices centered on `hint_index`
// (wrapping around for closed loops); pass search_window=0 to force a full
// linear scan (e.g. on the very first call, before a hint is meaningful).
TrackingErrorResult computeTrackingError(
  const std::vector<hunter_msgs::msg::Waypoint> & waypoints, bool is_closed_loop,
  const Pose2D & pose, std::size_t hint_index, std::size_t search_window);

}  // namespace hunter_control_common

#endif  // HUNTER_CONTROL__HUNTER_CONTROL_COMMON_HPP_
