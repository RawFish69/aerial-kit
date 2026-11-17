# Pilot stick command for a multirotor, interpreted by the backend's flight mode.
#
# Sticks are normalised: roll/pitch/yaw in [-1, 1], throttle in [0, 1].
#   pitch +1   stick forward: nose down / fly forward
#   roll  +1   right
#   yaw   +1   clockwise seen from above (nose right)
#   throttle   0.5 = hover thrust (ANGLE, ACRO) or zero climb rate (VELOCITY)
#
# The plant obeys this only while it is fresh (manual_timeout_sec), active,
# enabled, and every value is finite; otherwise it falls back to cmd_twist or
# to a position hold (motors idle only when already resting on the ground).
uint8 MODE_VELOCITY=0
uint8 MODE_ANGLE=1
uint8 MODE_ACRO=2

std_msgs/Header header
uint32 sequence_id
uint8 mode
float32 roll
float32 pitch
float32 yaw
float32 throttle
bool active
