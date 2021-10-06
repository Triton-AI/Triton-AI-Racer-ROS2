"""
Haoru Xue | July 2021 | hxue@ucsd.edu
"""

from simple_pid import PID
import numpy as np
import json

from rclpy.node import Node

DEFAULT_PID_CONFIG = {
    "steering": {
        "kp": 3.0,
        "ki": 0.2,
        "kd": 1.5
    },
    "throttle": {
        "kp": 3.0,
        "ki": 0.2,
        "kd": 2.0
    }
}

DEFAULT_WAYPOINT_CONFIG = {
    "waypoint_data_file": "waypoints.json",
    "n_look_ahead": 10
}


class WaypointFollower:
    """
    PID Pilot Mark II. This class does not require the track to be non-self-intersecting.
    """

    def __init__(self, cfg_pid, cfg_waypoint, node:Node):
        str_cfg = cfg_pid['steering']
        spd_cfg = cfg_pid['throttle']

        # Input CTE to optimal racing line; Output steering
        self.str_pid = PID(str_cfg['kp'], str_cfg['ki'], str_cfg['kd'],
                           setpoint=0.0, output_limits=(-1.0, 1.0), sample_time=0.05)
        # Input difference between the current speed and desired speed; Output throttle
        self.spd_pid = PID(spd_cfg['kp'], spd_cfg['ki'], spd_cfg['kd'],
                           setpoint=0.0, output_limits=(-0.1, 1.0), sample_time=0.05)
        self.node_ = node

        self.k_ = 10000

        wp_data_path = cfg_waypoint['waypoint_data_file']
        with open(wp_data_path, 'r') as input_file:
            self.wps = json.load(input_file)

        self.wp_coords = np.array(
            [(pt['lat'] * (self.k_), pt['lon'] * (-self.k_)) for pt in self.wps])
        # self.wp_headings = np.array([pt['heading'] for pt in self.wps])
        self.wp_speeds = np.array([pt['speed'] for pt in self.wps])

        self.last_nearest_wp_idx = None
        self.n_look_ahead = cfg_waypoint['n_look_ahead']

    def step(self, tele):

        if tele is not None:
            speed = tele['speed']
            heading = tele['heading']
            coord = tele['lat'] * (self.k_), tele['lon'] * (-self.k_)

            target_heading, target_speed = self.__plan(coord)
            #self.node_.get_logger().info("{:.4f}, {:.4f}, {:d}".format(
            #    target_heading, heading, self.last_nearest_wp_idx))
            return self.__act(heading, target_heading, speed, target_speed)
        return 0.0, 0.0, self

    def __findNearestWaypoint(self, coord, waypoints=None):
        """
        coord: (x, y)
        waypoints: np.array (N * 2)

        Exhaustively find the nearest waypoint.
        Return: the index of the nearest neighbor in self.wps
        """
        if waypoints is None:
            waypoints = self.wp_coords
        coord_arr = np.expand_dims(np.array(coord), axis=0)
        dists = coord_arr - waypoints
        return np.argmin(np.linalg.norm(dists, axis=1))

    def __lookAhead(self, idx, n):
        """
        Return the indices of the n waypoints ahead (including the current one), 
        considering the end and begining of a lap 
        """
        num_wps = len(self.wps)
        if num_wps - idx >= n:
            return np.linspace(idx, idx+n, n, endpoint=False, dtype=int)
        else:
            return np.hstack(
                (np.linspace(idx, num_wps, num_wps-idx, endpoint=False, dtype=int),
                 np.linspace(0, n-(num_wps-idx), n-(num_wps-idx),
                             endpoint=False, dtype=int)
                 )
            )

    def __lookBack(self, idx, n):
        """
        Return the indices of the n waypoints back (including the current one), 
        considering the end and begining of a lap 
        """
        num_wps = len(self.wps)
        if n <= idx+1:
            return np.linspace(idx, idx-n, n, endpoint=False, dtype=int)
        else:
            return np.hstack(
                (np.linspace(idx, 0, idx+1, endpoint=True, dtype=int),
                 np.linspace(num_wps-1, num_wps-(n-idx), n -
                             idx-1, endpoint=False, dtype=int)
                 )
            )

    def __efficientFindNearestWaypoint(self, coord, last_waypoint_idx):
        """
        coord: (x, y)
        last_waypoint_idx: int

        Efficiently find the nearest waypoint when possible, given the last nearest waypoint
        Return: the index of the nearest neighbor in self.wps
        """
        roi = 40
        nearby_wp_indices = self.__getNeighbors(last_waypoint_idx, roi)
        nearest_wp_idx = nearby_wp_indices[self.__findNearestWaypoint(
            coord, self.wp_coords[nearby_wp_indices])]

        # If the nearest neighbor is at the boundary of the ROI, we need to exhaustively search.
        if nearest_wp_idx == nearby_wp_indices[0] or nearest_wp_idx == nearby_wp_indices[-1]:
            return self.__findNearestWaypoint(coord)
        else:
            return nearest_wp_idx

    def __getNeighbors(self, idx, roi):
        """ Return roi * index of neighbor waypoints"""
        result = np.hstack(
            (np.flip(self.__lookBack(idx, roi)), self.__lookAhead(idx, roi)))
        return np.hstack((result[:roi-1], result[roi:]))

    def __getTargetHeading(self, target_wp_idx, coord):
        """
        Return the target heading in degree
        """
        target_wp_coord = self.wp_coords[target_wp_idx]
        target_heading = np.degrees(np.arctan2(
            target_wp_coord[1]-coord[1], target_wp_coord[0]-coord[0]))
        if target_heading < 0:
            target_heading = 360 + target_heading
        return target_heading

    def __plan(self, coord):

        #self.last_nearest_wp_idx = self.__findNearestWaypoint(coord)
        # OR

        if self.last_nearest_wp_idx is None:
            self.last_nearest_wp_idx = self.__findNearestWaypoint(coord)
        else:
            self.last_nearest_wp_idx = self.__efficientFindNearestWaypoint(
                coord, self.last_nearest_wp_idx)

        target_wp_idx = self.__lookAhead(
            self.last_nearest_wp_idx, self.n_look_ahead)[-1]
        target_heading = self.__getTargetHeading(
            target_wp_idx, coord)
        target_speed = self.wp_speeds[target_wp_idx]

        return target_heading, target_speed

    def __act(self, current_heading, target_heading, current_speed, target_speed):

        heading_diff = self.__calc_heading_difference(
            current_heading, target_heading)  # in [-180, 180]
        # print(heading_diff)
        heading_diff_normalized = heading_diff / 180  # normalize to [-1, 1]

        str = self.str_pid(heading_diff_normalized)

        d_spd = current_speed - target_speed
        thr = self.spd_pid(d_spd)

        return str, thr, self

    def __calc_heading_difference(self, curr_h, pred_h):
        # negative for turning left, positive for tunring right
        if pred_h > curr_h:
            tl = pred_h - curr_h
            tr = curr_h + 360 - pred_h
            return -tl if tl < tr else tr
        else:
            tl = 360 - curr_h + pred_h
            tr = curr_h - pred_h
            return -tl if tl < tr else tr

    def getName(self):
        return 'Waypoint Follower'