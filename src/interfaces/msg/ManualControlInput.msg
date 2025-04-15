float32 roll         # Left stick X-axis
float32 pitch        # Left stick Y-axis (inverted for correct mapping)
float32 yaw_velocity # Right stick X-axis
float32 thrust       # Right stick Y-axis, mapped from [-1,1] to [0,1]
uint8 arm            # 0 when not armed and 1 when armed
uint8 estop          # inactive when 0 and active when 1
uint8 selfdestruct   #yeps
