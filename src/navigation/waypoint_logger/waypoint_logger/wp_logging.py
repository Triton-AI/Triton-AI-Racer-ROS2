from abc import abstractclassmethod
import threading, json, enum, time
from os import path
from rclpy.node import Node


class LogState(enum.IntEnum):
    NotStarted = 0
    InProgress = 1
    Paused = 2
    Terminated = 3


class WaypointLogger:

    def __init__(self, log_hz, node:Node):

        self.log_hz = log_hz
        self.state = LogState.NotStarted
        self.t_log = None
        self.json_io = JsonFileIO()
        self.to_encode_ = []
        self.len_ = 0
        self.node_ = node
        # Logged variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.speed = 0.0

        self.pos_lock_ = threading.Lock()
        self.speed_lock_ = threading.Lock()


    def get_state(self, print_state=False):
        if print_state:
            self.node_.get_logger().info(f"Log State: {self.state.name}")
        return self.state


    def start_logging(self, file_name:str):
        file_name = path.abspath(file_name)
        self.state = LogState.InProgress
        self.node_.get_logger().info(f"Logging started. Waypoints are saved to {file_name}")
        self.t_log = threading.Thread(target=self.logging_thread_, daemon=True, args=[file_name])
        self.t_log.start()

    def pause_logging(self):
        if self.state == LogState.InProgress:
            self.state = LogState.Paused
            self.node_.get_logger().info("Logging paused.")
        else:
            self.node_.get_logger().warning("Cannot pause logging: it is not in progress.")


    def terminate_logging(self):
        if self.state == LogState.InProgress or self.state == LogState.Paused:
            self.state = LogState.Terminated
            self.node_.get_logger().info("Logging termination signaled.")
        elif self.state == LogState.Terminated:
            self.node_.get_logger().warning("Logging already terminated.")
        elif self.state == LogState.NotStarted:
            self.node_.get_logger().warning("Logging cannot be terminated: it has not started.")


    def resume_logging(self):
        if self.state == LogState.Paused:
            self.state = LogState.InProgress
            self.node_.get_logger().info("Logging resumed.")
        elif self.state == LogState.InProgress:
            self.node_.get_logger().info("Logging is already active.")
        elif self.state == LogState.Terminated:
            self.node_.get_logger().warning("Logging cannot be resumed: it has been terminated.")
        elif self.state == LogState.NotStarted:
            self.node_.get_logger().warning("Logging cannot be resumed: it has not been started.")


    def update_pose(self, x:float, y:float, z:float):
        self.pos_lock_.acquire()
        self.x, self.y, self.z = x, y, z
        self.pos_lock_.release()
   

    def update_speed(self, speed:float):
        self.speed_lock_.acquire()
        self.speed = speed
        self.speed_lock_.release()

    def get_current_waypoint(self):
        self.pos_lock_.acquire()
        dic = {"x": self.x, "y": self.y, "z": self.z}
        self.pos_lock_.release()

        self.speed_lock_.acquire()
        dic["speed"] = self.speed
        self.speed_lock_.release()

        return dic


    def logging_thread_(self, file_name:str):

        sleep_s = 1 / self.log_hz
        while True:
            if self.state == LogState.InProgress:
                self.to_encode_.append(self.get_current_waypoint())
                self.to_encode_[-1]["index"] = len(self.to_encode_) - 1
            elif self.state == LogState.Paused:
                pass
            elif self.state == LogState.Terminated:
                break
            time.sleep(sleep_s)
        self.node_.get_logger().info("Termination received. Writing to file.")
        # JsonFileIO.write_to_file(file_name, self.to_encode_)
        with open(file_name, 'w') as f:
            json.dump(self.to_encode_, f, indent=4)
        
        self.node_.get_logger().info(f"Waypoints outputed to {file_name}")


    def __del__(self):
        self.state = LogState.Terminated
        if self.t_log:
            self.t_log.join()


class JsonFileIO:
    @abstractclassmethod
    def write_to_file(file_name:str, json_packet):
        with open(file_name, 'w') as f:
            json.dump(json_packet, f, indent=4)
