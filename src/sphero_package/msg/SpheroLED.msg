# Message for controlling Sphero LED color

# LED type constants
uint8 LED_MAIN = 0   # Main LED (dome/body)
uint8 LED_FRONT = 1  # Front LED
uint8 LED_BACK = 2   # Back LED

uint8 led      # LED type (LED_MAIN, LED_FRONT, or LED_BACK)
uint8 red      # Red component (0-255)
uint8 green    # Green component (0-255)
uint8 blue     # Blue component (0-255)
