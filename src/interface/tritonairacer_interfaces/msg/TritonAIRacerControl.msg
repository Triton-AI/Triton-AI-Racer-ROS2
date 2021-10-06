std_msgs/Header header

tritonairacer_interfaces/Throttle throttle 
# [0, 100] throttle paddle percentage

tritonairacer_interfaces/Brake brake 
# [0, 100] brake paddle percentage

tritonairacer_interfaces/Steer steering_openloop
# [-1, 1] open-loop steering where negative is left
#         and positive is right

tritonairacer_interfaces/Steer steering_rad 
#[-pi, pi] close-loop steering where negative is left 
#          and positive is right