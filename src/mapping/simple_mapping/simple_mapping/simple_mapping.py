from dataclasses import dataclass

import cv2
import numpy as np

WALL = 255
SPACE = 0
UNKNOWN = 128


class SimpleMappingConfig(dataclass):
    grid_size_m: float = 0.05
    max_displacement_m: float = 0.2
    max_rotation_deg: float = 10.0
    map_block_size_m: float = 5.0


class SimpleMapping:
    def __init__(self, config: SimpleMappingConfig, debug_callback=print) -> None:
        self.config = config
        self.map = None
        self.curr_row = None
        self.curr_col = None
        self.curr_yaw = None

        self._initial_updated = False
        self._add_map_block("")

    def _add_map_block(self, edge):
        map_block_size = self.config.map_block_size_m / self.config.max_displacement_m
        if self.map is None:
            self.map = np.full(
                (map_block_size, map_block_size), UNKNOWN, dtype=np.uint8)
            self.curr_row = self.map.shape[0] // 2
            self.curr_col = self.map.shape[1] // 2
            self.curr_yaw = 0.0

        if edge == "top":
            block_to_append = np.full(
                (map_block_size, self.map.shape[1]), UNKNOWN, dtype=np.uint8)
            self.map = np.vstack([block_to_append], self.map)
        elif edge == "bottom":
            block_to_append = np.full(
                (map_block_size, self.map.shape[1]), UNKNOWN, dtype=np.uint8)
            self.map = np.vstack([self.map, block_to_append])
        elif edge == "left":
            block_to_append = np.full(
                (self.map.shape[0], map_block_size), UNKNOWN, dtype=np.uint8)
            self.map = np.hstack([block_to_append, self.map])
        elif edge == "right":
            block_to_append = np.full(
                (self.map.shape[0], map_block_size), UNKNOWN, dtype=np.uint8)
            self.map = np.hstack([self.map, block_to_append])

    def _update_map(self, scan_grid: np.ndarray):
        pass
    # TODO
