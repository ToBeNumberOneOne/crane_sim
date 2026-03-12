import zmq
import matplotlib.pyplot as plt
from collections import deque

N = 300  # 滚动窗口点数

# ZMQ 订阅（CONFLATE 必须在 connect 之前设置，否则不生效）
ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.setsockopt(zmq.CONFLATE, 1)   # ← 必须在 connect 之前，始终取最新帧
sock.setsockopt(zmq.RCVTIMEO, 100) # 100 ms 超时，保持图表响应
sock.connect("tcp://localhost:5555")
sock.setsockopt_string(zmq.SUBSCRIBE, "")

# 数据缓冲
AXES = ['x', 'y', 'z']
buf = {k: deque(maxlen=N) for k in
       ['t',
        'pos_x', 'pos_y', 'pos_z',
        'vel_x', 'vel_y', 'vel_z',
        'cmd_x', 'cmd_y', 'cmd_z',
        'sw_x',  'sw_y',
        'aw_x',  'aw_y']}
t0 = None

# ── 画布 ──────────────────────────────────────────────────────────────────────
plt.ion()
fig, axs = plt.subplots(4, 1, figsize=(12, 10), tight_layout=True)
fig.suptitle('Crane Real-time Monitor', fontsize=13)

GROUP_LABELS = [
    'Position (m)',
    'Velocity (m/s)  [solid=actual  dashed=cmd]',
    'Swing Angle (rad)',
    'Angular Velocity (rad/s)',
]
for ax, title in zip(axs, GROUP_LABELS):
    ax.set_title(title, loc='left', fontsize=9)
    ax.grid(True, alpha=0.4)
axs[-1].set_xlabel('Time (s)')

COLORS = {'x': 'C0', 'y': 'C1', 'z': 'C2'}

# 位置
lpos = {n: axs[0].plot([], [], color=COLORS[n], label=n.upper())[0] for n in AXES}
axs[0].legend(loc='upper left', fontsize=8)

# 速度（实际=实线，指令=虚线，同轴同色）
lvel = {n: axs[1].plot([], [], color=COLORS[n], label=f'{n.upper()} act')[0] for n in AXES}
lcmd = {n: axs[1].plot([], [], '--', color=COLORS[n], alpha=0.6, label=f'{n.upper()} cmd')[0] for n in AXES}
axs[1].legend(loc='upper left', fontsize=8, ncol=2)

# 摆角
lsw_x, = axs[2].plot([], [], color='C3', label='swing_x')
lsw_y, = axs[2].plot([], [], color='C4', label='swing_y')
axs[2].legend(loc='upper left', fontsize=8)

# 角速度
law_x, = axs[3].plot([], [], color='C3', label='ang_vel_x')
law_y, = axs[3].plot([], [], color='C4', label='ang_vel_y')
axs[3].legend(loc='upper left', fontsize=8)

# ── 主循环 ────────────────────────────────────────────────────────────────────
while plt.get_fignums():
    try:
        msg = sock.recv_json()
        print(f"Received state at t={msg['timestamp']:.2f}s: pos={msg['pos']} cmd_vel={msg['cmd_vel']} swing={msg['swing']}")
    except zmq.Again:
        plt.pause(0.05)
        continue

    # 时间基准
    ts = msg['timestamp']
    if t0 is None:
        t0 = ts
    buf['t'].append(ts - t0)

    # 填充缓冲
    for n in AXES:
        buf[f'pos_{n}'].append(msg['pos'][n])
        buf[f'vel_{n}'].append(msg['vel'][n])
        buf[f'cmd_{n}'].append(msg['cmd_vel'][n])
    buf['sw_x'].append(msg['swing']['swing_x'])
    buf['sw_y'].append(msg['swing']['swing_y'])
    buf['aw_x'].append(msg['swing']['ang_vel_x'])
    buf['aw_y'].append(msg['swing']['ang_vel_y'])

    t = list(buf['t'])

    # 更新曲线
    for n in AXES:
        lpos[n].set_data(t, list(buf[f'pos_{n}']))
        lvel[n].set_data(t, list(buf[f'vel_{n}']))
        lcmd[n].set_data(t, list(buf[f'cmd_{n}']))
    lsw_x.set_data(t, list(buf['sw_x']))
    lsw_y.set_data(t, list(buf['sw_y']))
    law_x.set_data(t, list(buf['aw_x']))
    law_y.set_data(t, list(buf['aw_y']))

    for ax in axs:
        ax.relim()
        ax.autoscale_view()

    plt.pause(0.02)
