# Generic parameter message
# Mainly used for emitting model parameter changes
std_msgs/Header header
int8 type # The parameter type (aero, env, prop, etc)
# typedef enum {
#     PARAM_TYPE_WORLD = 0,
#     PARAM_TYPE_ENV,
#     PARAM_TYPE_INIT,
#     PARAM_TYPE_INERTIAL,
#     PARAM_TYPE_AERO,
#     PARAM_TYPE_PROP,
#     PARAM_TYPE_GROUND
# } ParamType_t;
string name # Parameter name
float64 value # Parameter value