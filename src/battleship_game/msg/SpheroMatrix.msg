# Message for controlling Sphero LED matrix (for Sphero BOLT)

string pattern    # Pattern to display (e.g., "smile", "sad", "heart", "arrow", or custom)
uint8[] matrix    # Custom 8x8 matrix (64 values, 0-255 for brightness)
uint8 red         # Color red component (0-255)
uint8 green       # Color green component (0-255)
uint8 blue        # Color blue component (0-255)
float32 duration  # Duration to display in seconds (0 = indefinite)
