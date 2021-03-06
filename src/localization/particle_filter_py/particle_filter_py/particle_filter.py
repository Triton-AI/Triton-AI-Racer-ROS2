from datetime import datetime
import numpy as np
from threading import Thread, Lock, Event
from dataclasses import dataclass
import time
import math


@dataclass
class ParticleFilterConfig:
    initial_position: tuple
    initial_orientation_deg: float
    max_orientation_change_deg: float = 10.0
    max_position_change_m: float = 1.0
    num_sample: int = 300
    reject_match_beyond_m: float = 10.0


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
        while True:
            # start = datetime.now()
            if self.sensor_input is None:
                time.sleep(0.01)
                continue
            self.sensor_input_lock.acquire()
            sensor_input = self.sensor_input.copy()
            self.sensor_input_lock.release()
            if self.sensor_input_event.is_set():
                pass
                self.sensor_input_event.clear()
            pos_sample, yaw_sample = self._sample(
                self.pos_range * range_ratio, self.yaw_range * range_ratio)
            pos_sample += np.expand_dims(self.position, axis=0)
            pos_sample = np.append(pos_sample, self.position[np.newaxis, :], axis=0)
            yaw_sample += self.yaw
            yaw_sample = np.append(yaw_sample, self.yaw)
            feature_transformed = self._transform(
                self.features, pos_sample, yaw_sample)
            belief, best_score, best_match = self._evaluate_feature_match(
                sensor_input, feature_transformed)

            new_pos = np.vstack([pos_sample[belief], self.position])
            new_yaw = np.vstack([yaw_sample[belief], self.yaw])
            self.position = np.average(new_pos, axis=0)
            self.yaw = np.average(new_yaw)
            if self.yaw > 2 * np.pi:
                self.yaw -= 2 * np.pi
            elif self.yaw < 0:
                self.yaw += 2*np.pi
            # duration = datetime.now() - start
            # self.debug_callback(str(duration))

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
        pos_x = np.random.normal(0, pos_range / 2, self.config.num_sample)
        pos_y = np.random.normal(0, pos_range / 2, self.config.num_sample)
        pos = np.hstack(
            [pos_x[:, np.newaxis], pos_y[:, np.newaxis]]).astype(np.float32)
        yaw = np.random.normal(
            0, yaw_range / 2, self.config.num_sample).astype(np.float32)
        return pos, yaw

    def _transform(self, to_transform: np.ndarray, pos_offsets: np.ndarray, yaw_offsets: np.ndarray):
        to_transform = to_transform[np.newaxis, ...]
        pos_offsets = pos_offsets[:, np.newaxis, :]
        translated = to_transform - pos_offsets

        rotation_matrix = np.zeros((len(yaw_offsets), 2, 2))
        yaw_cos = np.cos(yaw_offsets)
        yaw_sin = np.sin(yaw_offsets)
        rotation_matrix[:, 0, 0] = yaw_cos
        rotation_matrix[:, 0, 1] = -yaw_sin
        rotation_matrix[:, 1, 1] = yaw_cos
        rotation_matrix[:, 1, 0] = yaw_sin
        rotated = translated @ rotation_matrix

        return rotated

    def _evaluate_feature_match(self, samples: np.ndarray, features: np.ndarray):
        # samples: num_sample * sample_length * 2
        samples = samples[np.newaxis, np.newaxis, :, :]
        features = features[:, :, np.newaxis, :]
        # num_sample * feature_length * sample_length * 2
        distances = np.linalg.norm(samples-features, axis=3)
        where_best_distances = np.argmin(distances, axis=2)
        best_distances = np.squeeze(np.take_along_axis(
            distances, where_best_distances[..., np.newaxis], axis=-1))
        valid_matches = best_distances < self.config.reject_match_beyond_m
        scores = np.sum(np.clip(1 / best_distances, 0.0, len(self.features)), axis=1, where=valid_matches)
        best_sample = np.nanargmax(scores)
        return best_sample, scores[best_sample], where_best_distances[best_sample]
