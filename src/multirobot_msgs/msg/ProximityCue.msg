# Proximity cue. Policy members CLUSTER to a target inter-robot spacing so an
# observer perceives them as one group. Coordination is per-unit and
# localization-fed: each unit computes its own target from the group positions
# and drives itself (DRIVE lane), ticked by a per-unit two-state timer SM.
float32 target_spacing_cm    # desired inter-robot spacing (equilibrium)
# Anchor the cluster seeks:
uint8 CENTROID=0             # mean of member positions (damped consensus)
uint8 FIELD_POINT=1         # a fixed field-frame point (trivially consistent)
uint8   anchor
float32 anchor_x             # used when anchor == FIELD_POINT
float32 anchor_y             # used when anchor == FIELD_POINT
float32 max_speed            # speed cap for each proximity step
float32 tick_hz              # per-unit control cadence (Hz); 2-5 typical
float32 min_separation_cm    # hard floor; never declare settled inside this
