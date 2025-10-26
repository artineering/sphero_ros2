# Message containing Sphero sensor data

# Orientation (in degrees)
float32 pitch
float32 roll
float32 yaw

# Accelerometer (in Gs)
float32 accel_x
float32 accel_y
float32 accel_z

# Gyroscope (in degrees per second)
float32 gyro_x
float32 gyro_y
float32 gyro_z

# Location
float32 x
float32 y

# Velocity
float32 velocity_x
float32 velocity_y

# Battery
uint8 battery_percentage

# Timestamp
builtin_interfaces/Time timestamp
