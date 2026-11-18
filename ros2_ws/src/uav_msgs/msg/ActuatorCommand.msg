# Direct actuator-level command for the twin-motor flying wing.
#
# Physical left/right order: thrust_left/elevon_left act on the LEFT (port)
# wing. Elevon angles are trailing-edge-up positive; equal positive elevons
# pitch nose-up, elevon_right - elevon_left > 0 rolls right. Thrust is per
# motor in newtons.
#
# The plant only obeys this while it is fresh (see the adapter's
# manual_timeout_sec), active, and every value is finite; it clamps all four
# values to the airframe spec's limits before they reach the simulator.
std_msgs/Header header
uint32 sequence_id
float32 thrust_left_n
float32 thrust_right_n
float32 elevon_left_rad
float32 elevon_right_rad
bool active
