std_msgs/Header header

float32 throttle 
# [0, 1] throttle paddle percentage

float32 brake 
# [0, 100] brake paddle percentage

float32 target_velocity
# Alternative to throttle and brake, set the velocity.

float32 steering_openloop
# [-1, 1] open-loop steering where positive is left
#         and negative is right

float32 steering_rad 
#[-pi, pi] close-loop steering where positive is left 
#          and negative is right
