# Status message for UWB-tracked Sphero robot
std_msgs/Header header
string sphero_name           # Sphero name (e.g., "SB-3660")
uint8 tag_id                 # UWB tag ID (1-12)
bool in_arena                # True if inside arena boundary
uint8 num_anchors_visible    # Number of anchors with fresh data
float64 x                    # X position in meters
float64 y                    # Y position in meters
float64 quality              # Position quality (residual RMSE)
