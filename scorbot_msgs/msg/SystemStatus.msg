# Status of one ESP32 Scorbot controller, published by scorbot_system_controller
# (~/status, 10 Hz by default). Values come straight from the hardware interface's
# "system" GPIO state interfaces; names are decoded from scorbot_protocol.

std_msgs/Header header

# scorbot.v1.SystemState: BOOT, UNHOMED, HOMING, READY, ACTIVE, FAULT
string state
uint8 state_code

# scorbot.v1.FaultCode; "none" when there is no latched fault
string fault
uint8 fault_code

# Joint names in controller order and per-joint flags
string[] joints
bool[] homed
bool[] limit_switch
uint32 homed_mask
uint32 limit_mask

# Link health: age of the newest JointState; true while under the timeout
float64 link_age_ms
bool link_ok

# Controller identity
uint32 boot_count
string firmware_version

# Result of the last home/enable/disable/clear_fault request
# (scorbot.v1.Result name; "pending" while one is in flight)
string last_result
int32 last_result_code

# Rolling count of JointCommands the controller dropped because it was not ACTIVE
uint32 commands_ignored
