# Live episode-termination signal for Phase 6 data collection.
#
# Published by episode_monitor and bagged, so `terminals` in the HDF5 is a
# RECORDED FACT rather than something re-derived offline from ground truth and
# world geometry. Re-deriving would make the world SDF a permanent dependency
# of every dataset: change the derivation and you silently relabel data you
# already collected.
#
# The same signal drives DAgger's terminate-and-respawn in Step 4, so the
# detection logic exists once rather than twice.

std_msgs/Header header

# True on the tick a crash is first detected. Edge, not level: a robot resting
# against a wall emits one event, not a stream.
bool terminal

# Which signal fired. Recorded because the two are deliberately redundant --
# contact sensors can miss a glancing scrape, and knowing which one caught a
# given crash is what tells you whether the redundancy is earning its keep.
uint8 TRIGGER_NONE=0
uint8 TRIGGER_CONTACT=1        # Gazebo contact sensor
uint8 TRIGGER_SCAN_PROXIMITY=2 # min(scan) < scan_contact_m
uint8 TRIGGER_BOTH=3
uint8 trigger

float32 min_scan_m       # closest LiDAR return this tick
uint32 contact_count     # contact points reported by the sensor this tick
