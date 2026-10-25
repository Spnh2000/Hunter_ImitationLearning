#include "hunter_control/hunter_control_common.hpp"

#include <algorithm>
#include <cmath>

namespace hunter_control_common
{

Pose2D poseFromOdometry(const nav_msgs::msg::Odometry & odom)
{
  Pose2D pose;
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;

  const auto & q = odom.pose.pose.orientation;
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  pose.yaw = std::atan2(siny, cosy);

  pose.speed = odom.twist.twist.linear.x;
  return pose;
}

double wrapToPi(double angle_rad)
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

TrackingErrorResult computeTrackingError(
  const std::vector<hunter_msgs::msg::Waypoint> & waypoints, bool is_closed_loop,
  const Pose2D & pose, std::size_t hint_index, std::size_t search_window)
{
  const std::size_t n = waypoints.size();

  auto squaredDist = [&](std::size_t i) {
    const double dx = pose.x - waypoints[i].x_m;
    const double dy = pose.y - waypoints[i].y_m;
    return dx * dx + dy * dy;
  };

  std::size_t best = hint_index < n ? hint_index : 0;
  double best_d2 = squaredDist(best);

  const bool full_scan = search_window == 0 || (search_window * 2 + 1) >= n;
  if (full_scan) {
    for (std::size_t i = 0; i < n; ++i) {
      const double d2 = squaredDist(i);
      if (d2 < best_d2) {
        best_d2 = d2;
        best = i;
      }
    }
  } else {
    const long n_signed = static_cast<long>(n);
    for (long offset = -static_cast<long>(search_window); offset <= static_cast<long>(search_window);
         ++offset) {
      long idx = static_cast<long>(hint_index) + offset;
      if (is_closed_loop) {
        idx = ((idx % n_signed) + n_signed) % n_signed;
      } else if (idx < 0 || idx >= n_signed) {
        continue;
      }
      const std::size_t i = static_cast<std::size_t>(idx);
      const double d2 = squaredDist(i);
      if (d2 < best_d2) {
        best_d2 = d2;
        best = i;
      }
    }
  }

  const auto & wp = waypoints[best];
  const double dx = pose.x - wp.x_m;
  const double dy = pose.y - wp.y_m;
  const double psi = wp.psi_rad;

  TrackingErrorResult result;
  result.index = best;
  // Left-normal of the path tangent (cos psi, sin psi) is (-sin psi, cos psi);
  // projecting the robot-relative vector onto it gives +ve when the robot is left of path.
  result.lateral_error_m = -dx * std::sin(psi) + dy * std::cos(psi);
  result.heading_error_rad = wrapToPi(pose.yaw - psi);
  result.path_s_m = wp.s_m;
  return result;
}

}  // namespace hunter_control_common
