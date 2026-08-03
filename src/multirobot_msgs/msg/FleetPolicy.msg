std_msgs/Header header
string   policy_id
bool     active               # false => REVOKE: members clear ALL cues
string[] members              # raw callsigns e.g. "SB-33E3"
bool     enable_similarity
bool     enable_proximity     # phase 2: cluster to target spacing (DRIVE lane)
bool     enable_common_fate   # accepted but not implemented this phase
SimilarityCue similarity
ProximityCue  proximity
# Synchronized start (optional). `now` is the coordinator's Unix epoch at
# broadcast; every member fires the cue at the shared absolute instant
# now + start_offset, regardless of when it receives the broadcast. Absent /
# zero (now <= 0.0 OR start_offset <= 0) => start immediately (back-compat).
float64  now                  # coordinator epoch, seconds since Unix epoch; 0 = unset
float64  start_offset         # seconds after `now` at which the cue fires; 0 = immediate
