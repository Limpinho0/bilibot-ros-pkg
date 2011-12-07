uint8 CHARGING_NOT_CHARGING = 0
uint8 CHARGING_CHARGING_RECOVERY = 1
uint8 CHARGING_CHARGING = 2
uint8 CHARGING_TRICKLE_CHARGING = 3
uint8 CHARGING_WAITING = 4 
uint8 CHARGING_CHARGING_ERROR = 5

Header header

#arm stuff
uint8 gripper_current
uint8 elbow_current
uint8 elbow_pos
bool gripper_closed
int8 gripper_vel
int8 elbow_vel
bool lower_limit
bool upper_limit


#battery_info
uint8 battery_voltage
int16 charging_current
bool charger_power_ind
bool charger_trickle
uint8 battery_charge_state

#create interface
bool create_charging
bool create_charge_enabled
bool create_on

#other info
uint8 gyro_raw
bool kinect_enabled
bool estop_button
bool demo_button