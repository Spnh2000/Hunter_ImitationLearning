# One point on a reference trajectory, expressed in the enclosing
# Trajectory message's header.frame_id. Field names mirror the
# f1tenth_racetracks raceline CSV columns (s_m, x_m, y_m, psi_rad,
# kappa_radpm, vx_mps, ax_mps2).

float64 s_m             # cumulative arc-length from waypoint 0 [m]
float64 x_m
float64 y_m
float64 psi_rad          # heading, REP103 yaw convention (CCW from +X)
float64 kappa_radpm      # signed path curvature [1/m], +ve = left turn
float64 vx_mps           # target longitudinal speed at this point [m/s]
float64 ax_mps2          # target longitudinal acceleration [m/s^2], informational
