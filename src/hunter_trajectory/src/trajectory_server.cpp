#include "hunter_trajectory/trajectory_server.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace hunter_trajectory
{

namespace
{

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::vector<std::string> split(const std::string & line, char delim)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, delim)) {
    tokens.push_back(trim(token));
  }
  return tokens;
}

double distance(const RawPoint & a, const RawPoint & b)
{
  return std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
}

// Central-difference heading; forward/backward difference at the endpoints.
void synthesizeHeadings(std::vector<RawPoint> & points)
{
  const size_t n = points.size();
  for (size_t i = 0; i < n; ++i) {
    const RawPoint & prev = points[i == 0 ? 0 : i - 1];
    const RawPoint & next = points[i + 1 == n ? n - 1 : i + 1];
    points[i].psi_rad = std::atan2(next.y_m - prev.y_m, next.x_m - prev.x_m);
  }
}

void synthesizeArcLength(std::vector<RawPoint> & points)
{
  double s = 0.0;
  points[0].s_m = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    s += distance(points[i - 1], points[i]);
    points[i].s_m = s;
  }
}

// Uniformly resample a closed-loop point sequence at step_m along its arc
// length, then apply `passes` rounds of a circular moving average of
// half-width halfwin points. Applied to centerline input only: the raw
// pixel-derived centerline's corners are sharper than the Hunter can steer
// (Monza's first chicane reaches kappa ~1.2 rad/m vs. the vehicle's
// tan(0.461)/0.65 = 0.77 rad/m full-lock limit), so the path itself must be
// rounded within the corridor. Window sized empirically on Monza: 2.8 m x2
// passes gives max kappa 0.56 with >=0.55 m wall clearance.
std::vector<RawPoint> resampleAndSmoothClosedLoop(
  const std::vector<RawPoint> & points, double step_m, int halfwin, int passes)
{
  // Uniform resample along the closed loop (segment from back to front included).
  std::vector<double> seg(points.size());
  double total = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    const RawPoint & a = points[i];
    const RawPoint & b = points[(i + 1) % points.size()];
    seg[i] = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    total += seg[i];
  }

  std::vector<RawPoint> out;
  out.reserve(static_cast<size_t>(total / step_m) + 1);
  double target = 0.0;
  size_t i = 0;
  double acc = 0.0;
  while (target < total) {
    while (acc + seg[i] < target) {
      acc += seg[i];
      i = (i + 1) % points.size();
    }
    const double f = seg[i] > 0.0 ? (target - acc) / seg[i] : 0.0;
    const RawPoint & a = points[i];
    const RawPoint & b = points[(i + 1) % points.size()];
    RawPoint p;
    p.x_m = a.x_m + f * (b.x_m - a.x_m);
    p.y_m = a.y_m + f * (b.y_m - a.y_m);
    out.push_back(p);
    target += step_m;
  }

  // Repeated circular moving average ~= Gaussian smoothing.
  const size_t n = out.size();
  for (int pass = 0; pass < passes; ++pass) {
    std::vector<RawPoint> next = out;
    for (size_t j = 0; j < n; ++j) {
      double sx = 0.0;
      double sy = 0.0;
      for (int k = -halfwin; k <= halfwin; ++k) {
        const RawPoint & q = out[(j + n + static_cast<size_t>(k + static_cast<int>(n))) % n];
        sx += q.x_m;
        sy += q.y_m;
      }
      const double m = 2.0 * halfwin + 1.0;
      next[j].x_m = sx / m;
      next[j].y_m = sy / m;
    }
    out = next;
  }
  return out;
}

// Central-difference curvature (dpsi/ds) from the already-synthesized headings
// and arc lengths; forward/backward difference at the endpoints. Gives
// centerline input the kappa_radpm that curvature feedforward and the
// curvature-limited speed profile need (raceline input ships its own).
void synthesizeCurvatures(std::vector<RawPoint> & points)
{
  const size_t n = points.size();
  for (size_t i = 0; i < n; ++i) {
    const RawPoint & prev = points[i == 0 ? 0 : i - 1];
    const RawPoint & next = points[i + 1 == n ? n - 1 : i + 1];
    const double ds = next.s_m - prev.s_m;
    if (ds <= 0.0) {
      points[i].kappa_radpm = 0.0;
      continue;
    }
    double dpsi = next.psi_rad - prev.psi_rad;
    while (dpsi > M_PI) {
      dpsi -= 2.0 * M_PI;
    }
    while (dpsi < -M_PI) {
      dpsi += 2.0 * M_PI;
    }
    points[i].kappa_radpm = dpsi / ds;
  }
}

}  // namespace

TrajectoryServer::TrajectoryServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_server", options)
{
  const std::string track_file = declare_parameter<std::string>("track_file", "");
  const std::string path_format = declare_parameter<std::string>("path_format", "auto");
  const std::string frame_id = declare_parameter<std::string>("frame_id", "odom");
  const std::string speed_source = declare_parameter<std::string>("speed_source", "curvature_limited");
  const double max_speed_mps = declare_parameter<double>("max_speed_mps", 1.0);
  const double min_speed_mps = declare_parameter<double>("min_speed_mps", 0.2);
  const double lateral_accel_limit_mps2 = declare_parameter<double>("lateral_accel_limit_mps2", 1.2);
  const double constant_speed_mps = declare_parameter<double>("constant_speed_mps", 0.5);
  const double close_loop_gap_threshold_m = declare_parameter<double>("close_loop_gap_threshold_m", 0.05);
  const bool visualize = declare_parameter<bool>("visualize", true);
  // World pose at which the odom frame originated. AckermannSteeringController's
  // wheel odometry always starts integrating from (0,0,yaw=0) -- it cannot know
  // where in the world the robot was spawned. The track CSV, however, is in
  // world coordinates. Publishing the CSV untransformed makes the tracker chase
  // a path rotated by the spawn yaw relative to reality (empirically: instant
  // ~86 deg heading error on Monza, full steering lock, robot pinned against
  // the wall while dead-reckoned diagnostics look converged). These parameters
  // must be set to the robot's spawn pose (benchmark.launch.py wires its
  // computed spawn_x/y/yaw in automatically); (0,0,0) means "CSV already in
  // odom frame" and preserves the old behavior.
  const double odom_origin_x_m = declare_parameter<double>("odom_origin_x_m", 0.0);
  const double odom_origin_y_m = declare_parameter<double>("odom_origin_y_m", 0.0);
  const double odom_origin_yaw_rad = declare_parameter<double>("odom_origin_yaw_rad", 0.0);
  // Centerline-only path smoothing window (0 disables). See
  // resampleAndSmoothClosedLoop() for why raw centerline corners are
  // infeasible for the Hunter's steering limit.
  declare_parameter<double>("centerline_smoothing_m", 2.8);

  if (track_file.empty()) {
    RCLCPP_FATAL(get_logger(), "Required parameter 'track_file' is not set.");
    throw std::runtime_error("trajectory_server: track_file parameter is required");
  }
  {
    std::ifstream check(track_file);
    if (!check.good()) {
      RCLCPP_FATAL(get_logger(), "track_file does not exist or is not readable: '%s'", track_file.c_str());
      throw std::runtime_error("trajectory_server: cannot open track_file");
    }
  }

  const TrackFormat format = detectFormat(track_file, path_format);
  std::vector<RawPoint> points =
    (format == TrackFormat::kCenterline) ? parseCenterline(track_file) : parseRaceline(track_file);

  if (points.size() < 2) {
    RCLCPP_FATAL(get_logger(), "Parsed fewer than 2 waypoints from '%s'; nothing to publish.", track_file.c_str());
    throw std::runtime_error("trajectory_server: track file did not yield a usable path");
  }

  const auto [is_closed_loop, total_length_m] = dedupeClosingPoint(points, close_loop_gap_threshold_m);

  // Centerline input no longer forces constant speed: parseCenterline()
  // synthesizes kappa_radpm (central-difference dpsi/ds), so the
  // curvature_limited profile works for both formats.
  const std::string effective_speed_source = speed_source;
  applySpeedProfile(
    points, effective_speed_source, min_speed_mps, max_speed_mps, lateral_accel_limit_mps2,
    constant_speed_mps);

  transformToOdomFrame(points, odom_origin_x_m, odom_origin_y_m, odom_origin_yaw_rad);

  const auto trajectory_msg = buildTrajectoryMsg(points, is_closed_loop, total_length_m, frame_id);

  rclcpp::QoS latched_qos(1);
  latched_qos.reliable().transient_local();

  trajectory_pub_ = create_publisher<hunter_msgs::msg::Trajectory>("/trajectory/reference", latched_qos);
  trajectory_pub_->publish(trajectory_msg);

  if (visualize) {
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/trajectory/path_viz", latched_qos);
    path_pub_->publish(buildPathMsg(trajectory_msg));
  }

  RCLCPP_INFO(
    get_logger(),
    "Published trajectory: %zu waypoints, %.2f m %s, frame_id='%s', speed_source='%s' (from '%s')",
    points.size(), total_length_m, is_closed_loop ? "closed loop" : "open path", frame_id.c_str(),
    effective_speed_source.c_str(), track_file.c_str());
}

TrackFormat TrajectoryServer::detectFormat(const std::string & path, const std::string & requested) const
{
  if (requested == "centerline") {
    return TrackFormat::kCenterline;
  }
  if (requested == "raceline") {
    return TrackFormat::kRaceline;
  }
  if (requested != "auto") {
    RCLCPP_FATAL(
      get_logger(), "Unknown path_format '%s'; expected 'auto', 'centerline', or 'raceline'.",
      requested.c_str());
    throw std::runtime_error("trajectory_server: invalid path_format");
  }

  std::string lower = path;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower.find("centerline") != std::string::npos) {
    return TrackFormat::kCenterline;
  }
  if (lower.find("raceline") != std::string::npos) {
    return TrackFormat::kRaceline;
  }
  RCLCPP_FATAL(
    get_logger(),
    "path_format='auto' could not detect the CSV format from filename '%s' (expected 'centerline' or "
    "'raceline' in the name); set path_format explicitly.",
    path.c_str());
  throw std::runtime_error("trajectory_server: could not auto-detect path_format");
}

std::vector<RawPoint> TrajectoryServer::parseCenterline(const std::string & path) const
{
  std::ifstream file(path);
  std::vector<RawPoint> points;
  std::string line;
  while (std::getline(file, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    const auto tokens = split(line, ',');
    if (tokens.size() < 2) {
      RCLCPP_WARN(get_logger(), "Skipping malformed centerline row: '%s'", line.c_str());
      continue;
    }
    RawPoint p;
    p.x_m = std::stod(tokens[0]);
    p.y_m = std::stod(tokens[1]);
    points.push_back(p);
  }

  const double smoothing_window_m = get_parameter("centerline_smoothing_m").as_double();
  if (smoothing_window_m > 0.0 && points.size() >= 4) {
    const double closing_gap =
      std::hypot(points.front().x_m - points.back().x_m, points.front().y_m - points.back().y_m);
    if (closing_gap < 5.0) {
      constexpr double kResampleStepM = 0.2;
      const int halfwin = std::max(1, static_cast<int>(std::lround(smoothing_window_m / 2.0 / kResampleStepM)));
      points = resampleAndSmoothClosedLoop(points, kResampleStepM, halfwin, 2);
      RCLCPP_INFO(
        get_logger(),
        "Centerline resampled to %zu points (%.2f m step) and smoothed (window %.2f m x2 passes) "
        "to keep corner curvature within the vehicle's steering limit.",
        points.size(), kResampleStepM, smoothing_window_m);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "centerline_smoothing_m=%.2f requested but the path is not a closed loop "
        "(closing gap %.2f m); skipping smoothing.",
        smoothing_window_m, closing_gap);
    }
  }

  synthesizeArcLength(points);
  synthesizeHeadings(points);
  synthesizeCurvatures(points);
  return points;
}

std::vector<RawPoint> TrajectoryServer::parseRaceline(const std::string & path) const
{
  std::ifstream file(path);
  std::vector<RawPoint> points;
  std::string line;
  while (std::getline(file, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    const auto tokens = split(line, ';');
    if (tokens.size() < 7) {
      RCLCPP_WARN(get_logger(), "Skipping malformed raceline row: '%s'", line.c_str());
      continue;
    }
    RawPoint p;
    p.s_m = std::stod(tokens[0]);
    p.x_m = std::stod(tokens[1]);
    p.y_m = std::stod(tokens[2]);
    p.psi_rad = std::stod(tokens[3]);
    p.kappa_radpm = std::stod(tokens[4]);
    p.vx_mps = std::stod(tokens[5]);
    p.ax_mps2 = std::stod(tokens[6]);
    points.push_back(p);
  }
  return points;
}

std::pair<bool, double> TrajectoryServer::dedupeClosingPoint(
  std::vector<RawPoint> & points, double gap_threshold_m) const
{
  const double closing_gap = distance(points.front(), points.back());

  // Two different closing conventions show up in the wild: raceline CSVs
  // repeat the first point as an exact-duplicate last row (closing_gap ~ 0);
  // centerline CSVs sample uniformly around the loop with *no* duplicate, so
  // the closing gap is approximately one ordinary segment length instead.
  // Testing only against a small fixed gap_threshold_m (sized for the first
  // case) misses the second entirely, so also compare against the local
  // segment length near the end of the file.
  const double last_segment_length = distance(points[points.size() - 2], points.back());

  const bool exact_duplicate = closing_gap < gap_threshold_m;
  const bool implicit_loop_closure = !exact_duplicate && closing_gap < last_segment_length * 1.5;
  const bool is_closed_loop = exact_duplicate || implicit_loop_closure;

  // exact_duplicate: the last row's own s_m already covers the full closed
  // distance back to the start. implicit_loop_closure: no such row exists,
  // so extend the arc length by the final (unsampled) closing segment.
  const double total_length_m =
    implicit_loop_closure ? points.back().s_m + closing_gap : points.back().s_m;

  if (exact_duplicate) {
    points.pop_back();  // only drop a genuine duplicate; keep real samples otherwise
  }
  return {is_closed_loop, total_length_m};
}

void TrajectoryServer::applySpeedProfile(
  std::vector<RawPoint> & points, const std::string & effective_speed_source, double min_speed_mps,
  double max_speed_mps, double lateral_accel_limit_mps2, double constant_speed_mps) const
{
  constexpr double kKappaEpsilon = 1e-3;  // rad/m; avoids div-by-zero on straights

  if (effective_speed_source == "constant") {
    const double v = std::clamp(constant_speed_mps, min_speed_mps, max_speed_mps);
    for (auto & p : points) {
      p.vx_mps = v;
    }
  } else if (effective_speed_source == "curvature_limited") {
    for (auto & p : points) {
      const double kappa = std::max(std::fabs(p.kappa_radpm), kKappaEpsilon);
      const double v = std::sqrt(lateral_accel_limit_mps2 / kappa);
      p.vx_mps = std::clamp(v, min_speed_mps, max_speed_mps);
    }
  } else {
    RCLCPP_FATAL(
      get_logger(), "Unknown speed_source '%s'; expected 'curvature_limited' or 'constant'.",
      effective_speed_source.c_str());
    throw std::runtime_error("trajectory_server: invalid speed_source");
  }
}

void TrajectoryServer::transformToOdomFrame(
  std::vector<RawPoint> & points, double origin_x_m, double origin_y_m,
  double origin_yaw_rad) const
{
  if (origin_x_m == 0.0 && origin_y_m == 0.0 && origin_yaw_rad == 0.0) {
    return;
  }
  const double c = std::cos(origin_yaw_rad);
  const double s = std::sin(origin_yaw_rad);
  for (auto & p : points) {
    const double dx = p.x_m - origin_x_m;
    const double dy = p.y_m - origin_y_m;
    p.x_m = c * dx + s * dy;
    p.y_m = -s * dx + c * dy;
    double psi = p.psi_rad - origin_yaw_rad;
    while (psi > M_PI) {
      psi -= 2.0 * M_PI;
    }
    while (psi < -M_PI) {
      psi += 2.0 * M_PI;
    }
    p.psi_rad = psi;
  }
  RCLCPP_INFO(
    get_logger(),
    "Transformed trajectory from world frame into odom frame (odom origin: x=%.3f, y=%.3f, yaw=%.3f rad)",
    origin_x_m, origin_y_m, origin_yaw_rad);
}

hunter_msgs::msg::Trajectory TrajectoryServer::buildTrajectoryMsg(
  const std::vector<RawPoint> & points, bool is_closed_loop, double total_length_m,
  const std::string & frame_id) const
{
  hunter_msgs::msg::Trajectory msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id;
  msg.is_closed_loop = is_closed_loop;
  msg.total_length_m = total_length_m;
  msg.waypoints.reserve(points.size());
  for (const auto & p : points) {
    hunter_msgs::msg::Waypoint wp;
    wp.s_m = p.s_m;
    wp.x_m = p.x_m;
    wp.y_m = p.y_m;
    wp.psi_rad = p.psi_rad;
    wp.kappa_radpm = p.kappa_radpm;
    wp.vx_mps = p.vx_mps;
    wp.ax_mps2 = p.ax_mps2;
    msg.waypoints.push_back(wp);
  }
  return msg;
}

nav_msgs::msg::Path TrajectoryServer::buildPathMsg(const hunter_msgs::msg::Trajectory & traj) const
{
  nav_msgs::msg::Path path;
  path.header = traj.header;
  path.poses.reserve(traj.waypoints.size());
  for (const auto & wp : traj.waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = traj.header;
    pose.pose.position.x = wp.x_m;
    pose.pose.position.y = wp.y_m;
    pose.pose.orientation.z = std::sin(wp.psi_rad / 2.0);
    pose.pose.orientation.w = std::cos(wp.psi_rad / 2.0);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace hunter_trajectory
