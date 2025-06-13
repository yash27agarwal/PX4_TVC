# This message contains information about motor speeds (rad/s) setpoints
# being sent to gz harmonic via GZbrige

uint64 timestamp				 # time since system start (microseconds)

int16 num_active_motors
float64[12] motor_speeds
