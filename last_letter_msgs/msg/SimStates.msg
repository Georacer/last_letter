#UAV States
Header header
Geoid geoid
geometry_msgs/Pose pose #NED
geometry_msgs/Twist velocity #body
Accelerations acceleration #body
float64[4] rotorspeed #in rad/s, NOTE: when not inialized at len=4, access causes segfault