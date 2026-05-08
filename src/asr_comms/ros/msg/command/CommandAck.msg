string command_type   # echoes the command that was acknowledged
uint8  result         # MAV_RESULT: 0=ACCEPTED 1=TEMP_REJECTED 2=DENIED 3=UNSUPPORTED 4=FAILED 6=CANCELLED
string message        # human-readable description of the result
