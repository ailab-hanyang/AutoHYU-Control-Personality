# std_msgs/Header
std_msgs/Header header

# reference information
autohyu_msgs/Reference reference

# novatel type
uint16 pos_type

# gnss
autohyu_msgs/WGS84 gnss
autohyu_msgs/WGS84 gnss_stdev

# position and motion
float64 x
float64 y
float64 z

float64 vx
float64 vy
float64 vz

float64 ax
float64 ay
float64 az

float64 roll
float64 pitch
float64 yaw

float64 roll_stdev
float64 pitch_stdev
float64 yaw_stdev

float64 roll_vel
float64 pitch_vel
float64 yaw_vel

# vehicle can
autohyu_msgs/VehicleCAN vehicle_can