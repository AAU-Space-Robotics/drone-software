float64 timestamp
#int8 id
#int8 mode
float64 position_timestamp
float32[] position #x, y, z
float64 velocity_timestamp
float32[] velocity #x, y, z
float32[] orientation  #roll, pitch, yaw
float32[] target_position #x, y, z
float32[] acceleration
float64 battery_state_timestamp
float32 battery_voltage     # in volts
float32 battery_current
float32 battery_percentage  # 0.0 to 100.0
float32 battery_discharged_mah
float32 battery_average_current
float32[] actuator_speeds #Speed of the four actuators
uint8 arming_state  
     
uint8 estop          
int16 flight_mode


