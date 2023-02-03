uint8 TARGET_LINK=0
uint8 TARGET_JOINT=1
uint8 TYPE_POSE=0
uint8 TYPE_VELOCITY=1
uint8 TYPE_ACCELERATION=2
uint8 TYPE_WRENCH=3

std_msgs/Header header
string[] name
uint8[] target_type
uint8[] command_type
CommandPayload[] values