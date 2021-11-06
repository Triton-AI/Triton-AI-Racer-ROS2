from PIL import Image
import os
import random
import json
import time
from io import BytesIO
import base64
import numpy as np
from gym_donkeycar.core.sim_client import SDClient


class TelemetryPack:
    def __init__(self, steering=0.0, throttle=0.0, speed=0.0,
                 pos_x=None, pos_y=None, pos_z=None, hit=None,
                 time=0.0, accel_x=None, accel_y=None, accel_z=None,
                 gyro_x=None, gyro_y=None, gyro_z=None, gyro_w=None,
                 pitch=None, yaw=None, roll=None, cte=None,
                 active_node=None, total_nodes=None, vel_x=None,
                 vel_y=None, vel_z=None, on_road=None,
                 progress_on_track=None):
        self.steering = steering
        self.throttle = throttle
        self.speed = speed
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
        self.hit = hit
        self.time = time
        self.accel_x = accel_x
        self.accel_y = accel_y
        self.accel_z = accel_z
        self.gyro_x = gyro_x
        self.gyro_y = gyro_y
        self.gyro_z = gyro_z
        self.gyro_w = gyro_w
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll
        self.cte = cte
        self.active_node = active_node
        self.total_nodes = total_nodes
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_z = vel_z
        self.on_road = on_road
        self.progress_on_track = progress_on_track

    def __str__(self) -> str:
        return json.dumps(self.__dict__)

class TelemetryInterface:
    def image_callback(self, img):
        pass
    def lidar_callback(self, lidar):
        pass
    def telemetry_callback(self, tele:TelemetryPack):
        pass

class GymInterface(SDClient):
    '''Talking to the donkey gym'''

    def __init__(self, poll_socket_sleep_time=0.01, debug=print, config={}, tele_callback=None):
        self.debug = debug
        self.lidar_config = config['lidar_config']
        self.cam_config = config['cam_config']
        self.racer_config = config['racer_config']
        self.car_config = config['car_config']
        self.sim_config = config['sim_config']

        self.gym_config = {}
        self.gym_config.update(self.lidar_config)
        self.gym_config.update(self.cam_config)
        self.gym_config.update(self.car_config)
        self.gym_config.update(self.racer_config)
        self.gym_config.update(self.sim_config)
        self.latency = self.gym_config['artificial_latency']

        self.tele_callback = tele_callback
        self.car_loaded = False
        SDClient.__init__(
            self, self.gym_config['host'], self.gym_config['port'], poll_socket_sleep_time=poll_socket_sleep_time)
        self.load_scene(self.gym_config['scene_name'])
        self.send_config()
        self.last_image = None

        self.lidar = None
        self.hsv = [0, 1, 1]

    def step(self, *args):
        steering = args[0]
        throttle = args[1]
        braking = args[2]
        if braking is None:
            braking = 0.0
        reset = args[3]
        self.send_controls(steering, throttle, braking)
        if reset:
            self.reset_car()

        return self.last_image, self.lidar, self.tele

    def onStart(self):
        self.debug(
            f'CAUTION: Confirm your artificial latency setting: {self.latency}ms.')

    def onShutdown(self):
        self.stop()

    def getName(self):
        return 'Gym Interface'

    def __try_get(self, dict, key, type_required):
        try:
            val = dict.get(key)
            return type_required(val)
        except:
            return None

    def on_msg_recv(self, json_packet):
        if json_packet['msg_type'] == "need_car_config":
            self.send_config()

        elif json_packet['msg_type'] == "car_loaded":
            self.debug('Car loaded.')

        elif json_packet['msg_type'] == "telemetry":
            if self.latency > 0.0:
                time.sleep(self.latency / 1000.0)
            imgString = json_packet.get("image")
            if imgString is not None:
                image = Image.open(BytesIO(base64.b64decode(imgString)))
                self.last_image = np.asarray(image, dtype=np.uint8)
                if self.tele_callback is not None:
                    self.tele_callback.image_callback(self.last_image)
            else:
                self.last_image = None

            # Telemetry, New since 21.04.05
            to_extract = ['steering_angle', 'throttle', 'speed', 'pos_x',
                          'pos_y', 'pos_z', 'hit', 'time',
                          'accel_x', 'accel_y', 'accel_z',
                          'gyro_x', 'gyro_y', 'gyro_z', 'gyro_w',
                          'pitch', 'yaw', 'roll',
                          'cte', 'activeNode', 'totalNodes',
                          'vel_x', 'vel_y', 'vel_z', 'on_road',
                          'progress_on_shortest_path']
            types = [float, float, float, float,
                     float, float, int, float,
                     float, float, float,
                     float, float, float, float,
                     float, float, float,
                     float, int, int,
                     float, float, float, int,
                     float]
            vals = []

            for i in range(len(to_extract)):
                vals.append(self.__try_get(
                    json_packet, to_extract[i], types[i]))

            self.tele = TelemetryPack(*tuple(vals))
            if self.tele_callback is not None:
                self.tele_callback.telemetry_callback(self.tele)

            if "lidar" in json_packet:
                self.lidar = json_packet["lidar"]
                if self.tele_callback is not None:
                    self.tele_callback.lidar_callback(self.lidar)

    def on_need_car_config(self):
        self.send_config()

    def send_config(self):
        '''
        send four config messages to set up car, racer, camera, and lidar
        '''
        self.debug('Sending configs...')
        self.debug('Sending racer info')
        # Racer info
        msg = {'msg_type': 'racer_info'}
        msg.update({str(key): str(val) for key, val in self.racer_config.items()})
        self.send_now(json.dumps(msg))

        time.sleep(1.0)

        self.debug('Sending car config')
        # Car config
        msg = {"msg_type": "car_config",
               "body_style": self.gym_config['body_style'],
               "body_r": self.gym_config['body_rgb'][0].__str__(),
               "body_g": self.gym_config['body_rgb'][1].__str__(),
               "body_b": self.gym_config['body_rgb'][2].__str__(),
               "car_name": self.gym_config['car_name'],
               "font_size": self.gym_config['font_size'].__str__()}
        self.send_now(json.dumps(msg))

        time.sleep(1.0)

        # Camera config
        if self.cam_config.get('enabled'):
            self.debug('Sending camera config')
            msg = {"msg_type": "cam_config"}
            msg.update({str(key): str(val) for key, val in self.gym_config.items()})
            self.send_now(json.dumps(msg))
            self.debug(
                f"Gym Interface: Camera resolution ({self.gym_config['img_w']}, {self.gym_config['img_h']}).")

        if self.lidar_config['enabled']:
            self.debug('Sending LiDAR config')
            msg = {'msg_type': "lidar_config"}
            msg.update({str(key): str(val) for key, val in self.lidar_config.items()})
            self.send_now(json.dumps(msg))

        time.sleep(1.0)
        self.debug('Done sending configs.')
        self.car_loaded = True

    def send_controls(self, steering, throttle, braking):
        msg = {"msg_type": "control",
               "steering": str(steering),
               "throttle": str(throttle),
               "brake": str(braking)}
        self.send_now(json.dumps(msg))

        ''' Would you like some RGB?
        import colorsys
        self.hsv[0] += 0.005
        if self.hsv[0] > 1 : self.hsv[0] = 0
        rgb = colorsys.hsv_to_rgb(*(tuple(self.hsv)))
        msg = { "msg_type" : "car_config",  
        "body_r" : int(rgb[0] * 255).__str__(), 
        "body_g" : int(rgb[1] * 255).__str__(), 
        "body_b" : int(rgb[2] * 255).__str__(),
        "body_style" : self.gym_config['body_style'], 
        "car_name" : self.gym_config['car_name'], 
        "font_size" : self.gym_config['font_size'].__str__() }
        self.send_now(json.dumps(msg))
        '''
        # this sleep lets the SDClient thread poll our message and send it out.
        # time.sleep(self.poll_socket_sleep_sec)

    def load_scene(self, scene):
        self.debug(f'Loading scene: {scene}')
        msg = {"msg_type": "load_scene", "scene_name": scene}
        self.send_now(json.dumps(msg))
        time.sleep(2.0)
        

    def reset_car(self):
        self.debug('Resetting car...')
        msg = {'msg_type': 'reset_car'}
        self.send(json.dumps(msg))

    def teleport(self, pos: tuple, quat: tuple):
        self.debug(f'Teleporting car to {pos}')
        pos_x, pos_y, pos_z = pos
        q_x, q_y, q_z, q_w = quat
        msg = {
            'msg_type': 'set_position',
            'pos_x': str(pos_x),
            'pos_y': str(pos_y),
            'pos_z': str(pos_z),
            'Qx': str(q_x),
            'Qy': str(q_y),
            'Qz': str(q_z),
            'Qw': str(q_w)
        }
        self.send(json.dumps(msg))
