# Frozen per-tick diagnostics contract every tracker node (pid_tracker,
# and Phase 4-9's pure_pursuit/stanley/lqr/mpc/rl trackers) must publish
# identically, so metrics_logger and summarize_run.py can benchmark any
# controller without knowing anything algorithm-specific about it.
#
# Lap completion is NOT tracked here -- metrics_logger derives it centrally
# from path_s_m wraparound against the Trajectory's total_length_m, so lap
# logic is never duplicated per-controller.

std_msgs/Header header
string  controller_name          # free-text tag (e.g. "pid"); never branch on this downstream
float64 lateral_error_m          # signed; +ve = robot left of path tangent
float64 heading_error_rad        # signed, wrapped to [-pi, pi]
float64 target_speed_mps
float64 actual_speed_mps
float64 speed_error_mps
float64 commanded_steering_rad   # post-clamp, actually sent this cycle
float64 commanded_speed_mps      # post-clamp, actually sent this cycle
float64 path_s_m                 # arc-length of nearest reference waypoint
uint32  closest_waypoint_index
