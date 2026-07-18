# Similarity cue. cue_type IDENTITY = steady visual; BEHAVIOR = shared behavior(s).
uint8 IDENTITY=0
uint8 BEHAVIOR=1
uint8 cue_type
# Visual (IDENTITY steady render, and the thing BEHAVIOR blink toggles):
uint8 red
uint8 green
uint8 blue
string  pattern      # optional named graphic; empty = none
uint8[] matrix       # optional custom 8x8; empty = none
# BEHAVIOR sub-behaviors (composable -- any combination when cue_type==BEHAVIOR):
bool    behavior_blink     # blink the visual on/off
float32 blink_hz           # blink rate (Hz)
bool    behavior_spin      # spin in place (DRIVE lane)
float32 spin_speed         # 0-255; 0 = handler default
float32 spin_duration_s    # 0 = long default
int32   spin_rotations     # 0 = use spin_duration_s
