float64 timestamp               # s

# Position controller output (FRD frame) — velocity correction added to traj feedforward
float32 pos_ctrl_x
float32 pos_ctrl_y
float32 pos_ctrl_z

# Position PID state per axis
float32 pos_err_x
float32 pos_err_y
float32 pos_err_z
float32 pos_int_x             # integrator state
float32 pos_int_y
float32 pos_int_z

# Combined velocity command sent to velocity controller (NED frame)
float32 vel_cmd_x
float32 vel_cmd_y
float32 vel_cmd_z

# Adaptive hover thrust estimate (evolves over flight)
float32 hover_thrust_estimate
