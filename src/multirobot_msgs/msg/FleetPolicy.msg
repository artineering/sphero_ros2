std_msgs/Header header
string   policy_id
bool     active               # false => REVOKE: members clear ALL cues
string[] members              # raw callsigns e.g. "SB-33E3"
bool     enable_similarity
bool     enable_proximity     # accepted but not implemented this phase
bool     enable_common_fate   # accepted but not implemented this phase
SimilarityCue similarity
