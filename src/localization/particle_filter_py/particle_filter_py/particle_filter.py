import numpy as np
from threading import Thread, Lock, Event
from dataclasses import dataclass
import time
import math


@dataclass
class ParticleFilterConfig:
    initial_position: tuple
    initial_orientation_deg: float
    max_orientation_change_deg: float = 5.0
    max_position_change_m: float = 2.0
    num_sample: int = 2000
    reject_match_beyond_m: float = 5.0


class ParticleFilter:
    def __init__(self, config: ParticleFilterConfig, features: np.ndarray, debug_callback=print) -> None:
        self.config = config
        self.features = features
        self.sensor_input = None
        self.sensor_input_lock = Lock()
        self.sensor_input_event = Event()
        self.debug_callback = debug_callback
        self.position = np.array(
            list(config.initial_position), dtype=np.float32)
        self.yaw = np.radians(config.initial_orientation_deg)
        self.yaw_range = np.radians(config.max_orientation_change_deg)
        self.pos_range = config.max_position_change_m
        self.t_mcl_ = Thread(target=self.mcl_thread_, daemon=True)
        self.t_mcl_.start()

    def update_lidar(self, points: np.ndarray):
        if isinstance(points, np.ndarray):
            self.sensor_input_lock.acquire()
            self.sensor_input = points.copy()
            self.sensor_input_event.set()
            self.sensor_input_lock.release()

    def get_pose(self):
        return self.position.copy(), self.quaternion_from_euler(0.0, 0.0, self.yaw)

    def mcl_thread_(self):
        range_ratio = 1.0
        last_min_distance = None
        while True:
            if self.sensor_input is None:
                time.sleep(0.01)
                continue
            self.sensor_input_lock.acquire()
            sensor_input = self.sensor_input.copy()
            self.sensor_input_lock.release()
            # if self.sensor_input_event.is_set():
            #     range_ratio = 1.0
            #     last_min_distance = None
            #     self.sensor_input_event.clear()
            pos_sample, yaw_sample = self._sample(self.pos_range * range_ratio, self.yaw_range * range_ratio)
            pos_sample += np.expand_dims(self.position, axis=0)
            yaw_sample += self.yaw
            sensor_transformed = self._transform(
                sensor_input, pos_sample, yaw_sample)
            sensor_transformed = np.swapaxes(sensor_transformed, 1, 2)
            belief, min_distance = self._evaluate_feature_match(sensor_transformed)
            if last_min_distance is not None:
                if min_distance < last_min_distance:
                    last_min_distance = min_distance
                    range_ratio *= 0.9
                    if range_ratio < 0.001:
                        range_ratio = 0.001
                else:
                    range_ratio *= 1.1
                    if range_ratio > 1.0:
                        range_ratio = 1.0
            else:
                last_min_distance = min_distance
            self.position = pos_sample[belief]
            self.yaw = yaw_sample[belief]

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion
        quat = [w, x, y, z]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def _sample(self, pos_range, yaw_range):
        pos = ((np.random.rand(self.config.num_sample, 2) * 2.0 - 1.0)
               * pos_range).astype(np.float32)
        yaw = ((np.random.rand(self.config.num_sample) * 2.0 - 1.0)
               * yaw_range).astype(np.float32)
        return pos, yaw

    def _transform(self, to_transform: np.ndarray, pos_offsets: np.ndarray, yaw_offsets: np.ndarray):
        rotation_matrix = np.zeros((len(yaw_offsets), 2, 2))
        yaw_cos = np.cos(yaw_offsets)
        yaw_sin = np.sin(yaw_offsets)
        rotation_matrix[:, 0, 0] = yaw_cos
        rotation_matrix[:, 0, 1] = -yaw_sin
        rotation_matrix[:, 1, 1] = yaw_cos
        rotation_matrix[:, 1, 0] = -yaw_sin
        rotated = rotation_matrix @ to_transform.T

        pos_offsets = pos_offsets[0, np.newaxis, 1]
        transformed = rotated + pos_offsets

        return transformed

    def _evaluate_feature_match(self, samples: np.ndarray):
        # samples: num_sample * sample_length * 2
        samples = samples[:, np.newaxis, :, :]
        features = self.features[np.newaxis, :, np.newaxis, :]
        # num_sample * feature_length * sample_length * 2
        distances = np.linalg.norm(samples-features, axis=3)
        best_distances = np.min(distances, axis=2)
        valid_matches = best_distances < self.config.reject_match_beyond_m
        ave_distances = np.sum(
            best_distances, axis=1, where=valid_matches)
        ave_distances /= np.sum(valid_matches, axis=1) 
        try:
            min_distance = np.nanargmin(ave_distances)
        except Exception as e:
            return -1, math.inf
        return min_distance, ave_distances[min_distance]
