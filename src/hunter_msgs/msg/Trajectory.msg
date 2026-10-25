# The authoritative reference path published by trajectory_server.
# Published transient_local so nodes started after trajectory_server
# still receive it. This message shape is frozen across Phases 3-9 --
# new algorithm-specific data goes on separate topics, never here.

std_msgs/Header header
Waypoint[] waypoints
bool is_closed_loop
float64 total_length_m   # arc-length of one full lap if is_closed_loop
