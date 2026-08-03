std_msgs/Header header
string   policy_id
bool     active               # false => REVOKE: members clear ALL cues
string[] members              # raw callsigns e.g. "SB-33E3"
bool     enable_similarity
bool     enable_proximity     # phase 2: cluster to target spacing (DRIVE lane)
bool     enable_common_fate   # accepted but not implemented this phase
SimilarityCue similarity
ProximityCue  proximity
