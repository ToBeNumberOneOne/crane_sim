import zmq
import threading
import time

class CraneState:
    def __init__(self):
        self.pos = {}
        self.vel = {}
        self.cmd_vel = {}
        self._data = None
        self._axes = None
        self._stop_event = threading.Event()

        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")

    def sample(self, data, axes):
        self._data = data
        self._axes = axes
        for name, axis in axes.items():
            self.cmd_vel[name] = round(float(axis.cur_v),3)
        self.pos["x"], self.pos["y"], self.pos["z"] = [round(float(x), 3) for x in data.qpos[:3]]
        self.vel["x"], self.vel["y"], self.vel["z"] = [round(float(x), 3) for x in data.qvel[:3]]
        

    def start_publish(self, interval=0.1):
        def loop():
            while not self._stop_event.is_set():
                if self._data and self._axes:
                    self.publish_state(self._data, self._axes)
                time.sleep(interval)
        threading.Thread(target=loop, daemon=True).start()

    def stop_publish(self):
        self._stop_event.set()

    def publish_state(self, data, axes):
        def to_float_list(arr):
            return [round(float(x), 3) for x in arr[:3]]
        msg = {
            "pos": to_float_list(data.qpos),
            "vel": to_float_list(data.qvel),
            "cmd_vel": [float(axis.cur_v) for axis in axes.values()]
        }
        self.socket.send_json(msg)