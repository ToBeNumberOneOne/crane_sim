import zmq
import matplotlib.pyplot as plt
from collections import deque

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

data = {
    "x": {"pos": deque(maxlen=600), "vel": deque(maxlen=600)},
    "y": {"pos": deque(maxlen=600), "vel": deque(maxlen=600)},
    "z": {"pos": deque(maxlen=600), "vel": deque(maxlen=600)},
}
t = deque(maxlen=600)
i = 0

plt.ion()
fig, axs = plt.subplots(6, 1, figsize=(10, 16))
lines = {}

for idx, name in enumerate(["x", "y", "z"]):
    # 位置
    l_pos, = axs[idx*2].plot([], [], label=f"{name.upper()} Position", color="b")
    axs[idx*2].set_ylabel(f"{name.upper()} Pos")
    axs[idx*2].legend()
    axs[idx*2].grid()
    # 速度
    l_vel, = axs[idx*2+1].plot([], [], label=f"{name.upper()} Velocity", color="r")
    axs[idx*2+1].set_ylabel(f"{name.upper()} Vel")
    axs[idx*2+1].legend()
    axs[idx*2+1].grid()
    lines[name] = (l_pos, l_vel)

axs[5].set_xlabel("Time (s)")

while True:
    msg = socket.recv_json()
    for idx, name in enumerate(["x", "y", "z"]):
        data[name]["pos"].append(msg["pos"][idx])
        data[name]["vel"].append(msg["vel"][idx])
    t.append(i * 0.1)
    i += 1

    for idx, name in enumerate(["x", "y", "z"]):
        lines[name][0].set_data(t, data[name]["pos"])
        lines[name][1].set_data(t, data[name]["vel"])
        axs[idx*2].relim()
        axs[idx*2].autoscale_view()
        axs[idx*2+1].relim()
        axs[idx*2+1].autoscale_view()

    plt.pause(0.01)