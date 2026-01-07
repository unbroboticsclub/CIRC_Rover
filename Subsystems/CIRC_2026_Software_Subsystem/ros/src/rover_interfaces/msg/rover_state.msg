# RoverState.msg
# High-level status of the rover

std_msgs/Header header          # Timestamp and frame info

float32 linear_velocity         # Forward/backward speed in m/s
float32 angular_velocity        # rotation rate in rad/s (left and right)

float32 battery_voltage         # Current battery voltage in Volts
float32 battery_percentage      # Battery level in percent (0-100)

bool emergency_stop             # True if e-stop is active
bool is_autonomous              # True if rover is in autonomous mode
