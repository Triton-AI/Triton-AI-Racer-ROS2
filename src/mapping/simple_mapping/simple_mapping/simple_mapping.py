from dataclasses import dataclass

import cv2
from numpy.lib.function_base import diff
from scipy import ndimage
from scipy.linalg import lu
import numpy as np
from yaml import scan

WALL = 0
SPACE = 255
UNKNOWN = 128


@dataclass
class SimpleMappingConfig:
    grid_size_m: float = 0.2
    max_displacement_m: float = 2.0
    max_rotation_deg: float = 15.0
    map_block_size_m: float = 40


class SimpleMapping:
    def __init__(self, config: SimpleMappingConfig, debug_callback=print) -> None:
        self.config = config
        self.map = None
        self.curr_row = None
        self.curr_col = None
        self.curr_x = None
        self.curr_y = None
        self.curr_yaw = None
        self.last_scan = None
        self.debug_callback = debug_callback
        self._initial_updated = False
        self._add_map_block("")

    def on_new_map_part_received(self, arr: np.ndarray):
        rotate_deg, trans_x, trans_y, arr_rotated = self._match_grid(arr)
        if None in (rotate_deg, trans_x, trans_y):
            self.debug_callback("Unable to match. Pointcloud dropped.")
            return
        yaw_rad = np.radians(self.curr_yaw)
        rotate_matrix = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad)],
            [np.sin(yaw_rad), np.cos(yaw_rad)],
        ])
        translation_matrix = np.dot(np.array([trans_x, trans_y]), rotate_matrix)
        self._update_map(arr_rotated, rotate_deg, translation_matrix)
        cv2.imshow("Map", self.map.astype(np.uint8))
        cv2.waitKey(1)
        self.last_scan = arr.copy()

    def _add_map_block(self, edge):
        map_block_size = int(self.config.map_block_size_m //
                             self.config.grid_size_m)
        if self.map is None:
            self.map = np.full(
                (map_block_size, map_block_size), UNKNOWN, dtype=np.uint32)
            self.curr_row = self.map.shape[0] // 2
            self.curr_y = float(map_block_size - self.curr_row)
            self.curr_col = self.map.shape[1] // 2
            self.curr_x = float(self.curr_col)
            self.curr_yaw = 0.0

        if edge == "top":
            block_to_append = np.full(
                (map_block_size, self.map.shape[1]), UNKNOWN, dtype=np.uint32)
            self.map = np.vstack([block_to_append, self.map])
            self.curr_row += map_block_size
        elif edge == "bottom":
            block_to_append = np.full(
                (map_block_size, self.map.shape[1]), UNKNOWN, dtype=np.uint32)
            self.map = np.vstack([self.map, block_to_append])
            self.curr_y += map_block_size
        elif edge == "left":
            block_to_append = np.full(
                (self.map.shape[0], map_block_size), UNKNOWN, dtype=np.uint32)
            self.map = np.hstack([block_to_append, self.map])
            self.curr_col += map_block_size
            self.curr_x += map_block_size
        elif edge == "right":
            block_to_append = np.full(
                (self.map.shape[0], map_block_size), UNKNOWN, dtype=np.uint32)
            self.map = np.hstack([self.map, block_to_append])

    def _auto_allocate_map(self, grid_to_add):
        map_h, map_w = self.map.shape[:2]
        half_h, half_w = grid_to_add.shape[:2]
        if self.curr_row <= half_h:
            self._add_map_block('top')
            map_h, map_w = self.map.shape[:2]
        if self.curr_row + half_h >= map_h:
            self._add_map_block('bottom')
            map_h, map_w = self.map.shape[:2]
        if self.curr_col <= half_w:
            self._add_map_block('left')
            map_h, map_w = self.map.shape[:2]
        if self.curr_col + half_w >= map_w:
            self._add_map_block('right')

    def _add_new_map_part(self, grid_to_add: np.ndarray):
        grid_h, grid_w = grid_to_add.shape[:2]
        grid_top_row = self.curr_row - grid_h // 2
        grid_top_col = self.curr_col - grid_w // 2

        grid_mask = grid_to_add != UNKNOWN
        np.copyto(self.map[grid_top_row: grid_top_row + grid_h, grid_top_col:
                  grid_top_col + grid_w], grid_to_add, where=grid_mask)

    def _create_scan_grid(self, scan_arr: np.ndarray):
        arr_valid_scan = scan_arr[~np.all(scan_arr == 0, axis=1)].astype(int)

        grid_size = int(self.config.map_block_size_m /
                        self.config.grid_size_m * 2)
        test_arr = np.full((grid_size, grid_size), UNKNOWN, dtype=np.uint8)
        arr_valid_scan = arr_valid_scan + grid_size // 2

        for x, y in arr_valid_scan:
            test_arr = cv2.line(
                test_arr, (grid_size // 2, grid_size // 2), (x, y), SPACE, 1)

        test_arr[arr_valid_scan[:, 1], arr_valid_scan[:, 0]] = WALL
        return np.flip(test_arr, axis=0)

    def _match_grid(self, arr: np.ndarray):
        if self.last_scan is None:
            return 0.0, 0.0, 0.0, arr.copy()
        test_scan = arr.copy()

        temp_yaw = 0.0
        temp_x = 0.0
        temp_y = 0.0
        mask = np.all(((self.last_scan!=0) & (test_scan !=0)), axis=1)
        for i in range(10):
            # Minimize rotation error 
            slope_kernel = np.array([[1], [0], [-1]])
            test_scan_slope_field = cv2.filter2D(np.vstack([test_scan[-1, np.newaxis], test_scan, test_scan[0, np.newaxis]]), cv2.CV_32F, slope_kernel)[1:-1]
            test_scan_slope = test_scan_slope_field[:,1] / test_scan_slope_field[:,0]
            last_scan_slope_field = cv2.filter2D(np.vstack([self.last_scan[-1, np.newaxis], self.last_scan, self.last_scan[0, np.newaxis]]), cv2.CV_32F, slope_kernel)[1:-1]
            last_scan_slope = last_scan_slope_field[:,1] / last_scan_slope_field[:,0]

            yaw_diff_rad = (np.tanh(test_scan_slope) - np.tanh(last_scan_slope))[mask]
            yaw_diff_rad = yaw_diff_rad[(yaw_diff_rad != np.nan) & (np.abs(yaw_diff_rad) < np.radians(self.config.max_rotation_deg))]

            rotate_rad = np.average(yaw_diff_rad)
            if np.isnan(rotate_rad):
                return None, None, None, None
            temp_yaw += np.degrees(rotate_rad)
            rotate_matrix = np.array([
                [np.cos(rotate_rad), -np.sin(rotate_rad)],
                [np.sin(rotate_rad), np.cos(rotate_rad)],
            ])
            test_scan = np.dot(test_scan, rotate_matrix)

            one_idx_equal_to_deg = 1.0
            idx_shift = int(np.degrees(rotate_rad) / one_idx_equal_to_deg)
            test_scan = np.roll(test_scan, idx_shift, axis=0)

            # Minimize translation error
            translate_dists = (self.last_scan - test_scan)[mask]
            translate_dists = translate_dists[np.linalg.norm(translate_dists, axis=1) < (self.config.max_displacement_m // self.config.grid_size_m)]
            move_x, move_y = np.average(translate_dists, axis=0)
            if np.isnan(move_x) or np.isnan(move_y):
                return None, None, None, None
            temp_x += move_x
            temp_y += move_y
            test_scan[:, 0] = test_scan[:, 0] + move_x
            test_scan[:, 1] = test_scan[:, 1] + move_y
            pass

        yaw_rad = np.radians(temp_yaw)
        arr_rotated = arr.copy()
        arr_rotated[:, 0] = arr[:, 0] - temp_x
        arr_rotated[:, 1] = arr[:, 1] - temp_y
        rotate_matrix = np.array([
                [np.cos(yaw_rad), -np.sin(yaw_rad)],
                [np.sin(yaw_rad), np.cos(yaw_rad)],
            ])
        arr_rotated = np.dot(arr, rotate_matrix)
        return -temp_yaw, temp_x, temp_y, arr_rotated

    def _update_map(self, arr_rotated: np.ndarray, rotation_deg, translation_m: np.ndarray):
        scan_grid = self._create_scan_grid(arr_rotated)
        self.curr_yaw += rotation_deg
        if self.curr_yaw > 360:
            self.curr_yaw -= 360
        elif self.curr_yaw < 0:
            self.curr_yaw += 360
        trans_x, trans_y = translation_m[:2]

        self.curr_x += trans_x
        self.curr_y += trans_y
        self.curr_row = round(self.map.shape[0] - self.curr_y)
        self.curr_col = round(self.curr_x)
        self._auto_allocate_map(scan_grid)
        self._add_new_map_part(scan_grid)
        print(self.curr_yaw)

if __name__ == "__main__":
    config = SimpleMappingConfig()
    mapping = SimpleMapping(config)

    # scan_1 = np.zeros((360, 2), dtype=np.float32)
    # scan_angles = np.linspace(0, 359, 360,)
    # for i in range(180, 225):
    #     scan_1[i] = np.array([-50, np.tan(np.radians(i)) * -50], dtype=np.float32)
    # for i in range(225, 315):
    #     scan_1[i] = np.array([-50 / np.tan(np.radians(i)), -50], dtype=np.float32)

    
    # yaw_rad = np.radians(10)
    # rotate_matrix = np.array([
    #     [np.cos(yaw_rad), -np.sin(yaw_rad)],
    #     [np.sin(yaw_rad), np.cos(yaw_rad)],
    # ])

    # scan_2 = np.dot(scan_1, rotate_matrix).astype(np.float32)
    # scan_2 = np.roll(scan_2, 5, axis=0)
    # scan_2[:, 0][np.all(scan_2 != 0, axis=1)] -= 8
    # scan_2[:, 1][np.all(scan_2 != 0, axis=1)] -= 6
    import pickle
    with open("scan_1.pkl", "rb") as f:
        scan_1 = pickle.load(f)
    with open("scan_2.pkl", "rb") as f:
        scan_2 = pickle.load(f)

    mapping.on_new_map_part_received(scan_1)
    cv2.imshow("Before", mapping.map.astype(np.uint8))
    cv2.waitKey(0)
    mapping.on_new_map_part_received(scan_2)    
    cv2.imshow("After", mapping.map.astype(np.uint8))
    cv2.waitKey(0)