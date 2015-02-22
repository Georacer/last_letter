# Geoid - Describe position/velocity on the geoid
float64 WGS84_Ra = 6378137.0 # Earth ellipsoid semi-major axis (alpha)
float64 EARTH_flattening = 0.003352811
float64 WGS84_e2 = 0.006694380022901
float64 EARTH_Omega = 7.292115e-5
float64 EARTH_grav = 9.7803267714

float64 latitude
float64 longitude
float64 altitude
geometry_msgs/Vector3 velocity #in m/s North, East, Up
