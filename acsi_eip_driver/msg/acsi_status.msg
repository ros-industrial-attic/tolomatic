#Drive Status from drive_status bits and remapped regs.
#current_position is unchanged from acsi_inputs
bool stopped
bool host_control
bool homed
bool enabled
bool moving
bool brake_off
bool in_position
float32 target_position
float32 position_error
float32 current_position
