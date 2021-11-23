std_msgs/Header header

uint8 longitudinal_control_type
uint8 LON_CTL_THROTTLE_BRAKE = 0
uint8 LON_CTL_SPEED = 1

float32 throttle 
# [0, 1] throttle paddle percentage

float32 brake 
# [0, 100] brake paddle percentage

float32 target_velocity
# Alternative to throttle and brake, set the velocity.

uint8 lateral_control_type
uint8 LAT_CTL_OPENLOOP = 0
uint8 LAT_CTL_CLOSELOOP = 1

float32 steering_openloop
# [-1, 1] open-loop steering where positive is left
#         and negative is right

float32 steering_rad 
#[-pi, pi] close-loop steering where positive is left 
#          and negative is right