# Information required to build a MAVLink HIL_STATE_QUATERNION message
std_msgs/Header header

# NED to Body-frame quaternion
geometry_msgs/Quaternion quaternion

# Body-frame acceleration in m/s/s
geometry_msgs/Vector3 acceleration

# Body-frame angular velocity in rad/s
geometry_msgs/Vector3 velocity_angular

# WGS84 coordinates in degrees and meters
float64 latitude
float64 longitude
float64 altitude

# ENU velocities in m/s
geometry_msgs/Vector3 gnss_velocity

# Airdata in m/s
float64 airspeed_indicated
float64 airspeed_true
