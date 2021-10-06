import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Joy
from geometry_msgs.msg import TwistStamped, Twist
from waypoint_interfaces.srv import TerminateWaypointLogging, ToggleWaypointLogging, GetWaypointLogState
from .wp_logging import WaypointLogger, LogState

import numpy as np
class WaypointLoggingNode(Node):
    def __init__(self):
        super().__init__("wp_log_node")
        self.declare_parameter('waypoint_logging_hz', 20)
        self.declare_parameter('default_save_file_name', 'waypoint.json')
        self.declare_parameter('js_toggle_log_button', 0)
        self.declare_parameter('js_terminate_log_button', 1)
        self.declare_parameter('js_get_log_state_button', 2)
        self.logging_hz_ = self.get_parameter("waypoint_logging_hz").get_parameter_value().integer_value
        self.logger_ = WaypointLogger(self.logging_hz_, self)

        # Pubs N' Subs
        self.gnss_sub_ = self.create_subscription(NavSatFix, "/fix", self.gnss_callback_, 1)
        self.vel_sub_ = self.create_subscription(TwistStamped, "/vel", self.vel_callback_, 1)
        self.joy_sub_ = self.create_subscription(Joy, "/joy", self.joy_callback_, 1)

        # ROS parameters
        self.default_file_name_ = self.get_parameter("default_save_file_name").get_parameter_value().string_value
        self.js_toggle_log_button_ = self.get_parameter("js_toggle_log_button").get_parameter_value().integer_value
        self.js_terminate_log_button = self.get_parameter("js_terminate_log_button").get_parameter_value().integer_value
        self.js_get_log_state_button = self.get_parameter("js_get_log_state_button").get_parameter_value().integer_value

        # ROS services
        self.toggle_log_srv_ = self.create_service(ToggleWaypointLogging, "/waypoint/toggle_log", 
                                                    self.toggle_log_callback_)
        self.terminate_log_srv_ = self.create_service(TerminateWaypointLogging, "/waypoint/terminate_log", 
                                                    self.terminate_log_callback_)
        self.get_state_srv_ = self.create_service(GetWaypointLogState, "/waypoint/get_log_state",
                                                    self.get_state_callback_)

        # Class members
        self.state_toggle_log_button_ = 0
        self.state_terminate_log_button_ = 0
        self.state_get_log_state_button_ = 0

    def gnss_callback_(self, msg:NavSatFix):
        if np.isnan(msg.altitude):
            msg.altitude = 0.0
        self.logger_.update_gnss(msg.latitude, msg.longitude, msg.altitude)


    def vel_callback_(self, msg:TwistStamped):
        self.logger_.update_speed(self.__twist_to_vel(msg.twist))


    def toggle_log_callback_(self, request:ToggleWaypointLogging.Request, response:ToggleWaypointLogging.Response):
        
        file_name = request.file_name
        response.state_code = int(self.toggle_log_(file_name))
        return response

    
    def joy_callback_ (self, msg:Joy):
        self.state_toggle_log_button_ = self.check_js_button_event_(self.state_toggle_log_button_, 
                                                                    msg.buttons[self.js_toggle_log_button_], 
                                                                    self.toggle_log_)
        self.state_terminate_log_button_ = self.check_js_button_event_(self.state_terminate_log_button_,
                                                                    msg.buttons[self.js_terminate_log_button],
                                                                    self.logger_.terminate_logging)
        self.state_get_log_state_button_ = self.check_js_button_event_(self.state_get_log_state_button_,
                                                                    msg.buttons[self.js_get_log_state_button],
                                                                    self.logger_.get_state, print_state=True)

    def terminate_log_callback_(self, request:ToggleWaypointLogging.Request, response:ToggleWaypointLogging.Response):

        self.logger_.terminate_logging()
        response.error = (self.logger_.get_state() != LogState.Terminated)
        return response


    def get_state_callback_(self, request:GetWaypointLogState.Request, response:GetWaypointLogState.Response):
        response.state_code = int(self.logger_.get_state(print_state=True))
        return response


    def check_js_button_event_(self, last_state: int, current_state: int, callback, **kwargs):
        if last_state is not None:
            if last_state != current_state:
                callback(**kwargs)
        return current_state

    def toggle_log_(self, custom_file_name=""):
        state = self.logger_.get_state()
        if state == LogState.InProgress:
            self.logger_.pause_logging()
        elif state == LogState.Paused:
            self.logger_.resume_logging()
        elif state == LogState.NotStarted:
            file_name = self.default_file_name_
            if custom_file_name:
                file_name = custom_file_name
            self.logger_.start_logging(file_name)
        return self.logger_.get_state()

    def __twist_to_vel(self, twist:Twist):
        lin_vel = np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=float)
        for i in range(len(lin_vel)):
            if np.isnan(lin_vel[i]):
                lin_vel[i] = 0.0
        return np.linalg.norm(lin_vel)



def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoggingNode()
    rclpy.spin(node)  
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()

    except Exception as e:
        print(str(e))
        raise e
