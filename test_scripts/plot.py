import zmq
import matplotlib.pyplot as plt
from collections import deque

N = 90

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.setsockopt(zmq.CONFLATE, 1)
sock.setsockopt(zmq.RCVTIMEO, 100)
sock.connect("tcp://localhost:5555")
sock.setsockopt_string(zmq.SUBSCRIBE, "")

AXES = ['x', 'y', 'z']

buf = {k: deque(maxlen=N) for k in
       ['t',
        'pos_x', 'pos_y', 'pos_z',
        'vel_x', 'vel_y', 'vel_z',
        'cmd_x', 'cmd_y', 'cmd_z',
        'sw_x', 'sw_y',
        'aw_x', 'aw_y']}

t0 = None

plt.ion()

# =========================
# Figure 1：运动控制
# =========================
fig1, axes1 = plt.subplots(3, 1, figsize=(14, 10), constrained_layout=True)
fig1.suptitle('Motion Control Monitor', fontsize=14, fontweight='bold')

# =========================
# Figure 2：防摇
# =========================
fig2, axes2 = plt.subplots(2, 1, figsize=(14, 6), constrained_layout=True)
fig2.suptitle('Anti-Sway Monitor', fontsize=14, fontweight='bold')

COLORS = {
    'pos': 'C0',
    'vel': 'C1',
    'cmd': 'C2',
    'sw': 'C3',
    'aw': 'C4'
}


# ─────────────────────────
# 工具函数
# ─────────────────────────
def setup_axis(ax, title):
    ax.set_title(title, fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time (s)')


def create_motion_plot(ax, axis_name):
    setup_axis(ax, f'{axis_name.upper()} Axis')

    lpos, = ax.plot([], [], color=COLORS['pos'], label='Position')
    ax.set_ylabel('Position (m)', color=COLORS['pos'])
    ax.tick_params(axis='y', labelcolor=COLORS['pos'])

    ax_v = ax.twinx()
    lvel, = ax_v.plot([], [], color=COLORS['vel'], label='Velocity')
    ax_v.set_ylabel('Velocity (m/s)', color=COLORS['vel'])
    ax_v.tick_params(axis='y', labelcolor=COLORS['vel'])

    ax_cmd = ax.twinx()
    ax_cmd.spines['right'].set_position(('outward', 60))
    lcmd, = ax_cmd.plot([], [], '--', color=COLORS['cmd'], label='Cmd')
    ax_cmd.set_ylabel('Cmd (m/s)', color=COLORS['cmd'])
    ax_cmd.tick_params(axis='y', labelcolor=COLORS['cmd'])

    lines, labels = [], []
    for a in [ax, ax_v, ax_cmd]:
        l, lab = a.get_legend_handles_labels()
        lines += l
        labels += lab
    ax.legend(lines, labels, loc='upper right', fontsize=8)

    return lpos, lvel, lcmd, ax_v, ax_cmd


def create_swing_plot(ax, axis_name):
    setup_axis(ax, f'{axis_name.upper()} Direction')

    lsw, = ax.plot([], [], color=COLORS['sw'], label='Swing')
    ax.set_ylabel('Angle (rad)', color=COLORS['sw'])
    ax.tick_params(axis='y', labelcolor=COLORS['sw'])

    ax_aw = ax.twinx()
    law, = ax_aw.plot([], [], '--', color=COLORS['aw'], label='Ang Vel')
    ax_aw.set_ylabel('Angular Velocity (rad/s)', color=COLORS['aw'])
    ax_aw.tick_params(axis='y', labelcolor=COLORS['aw'])

    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax_aw.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)

    return lsw, law, ax_aw


# ─────────────────────────
# 创建图1（XYZ）
# ─────────────────────────
lpx, lvx, lcx, axvx, axcx = create_motion_plot(axes1[0], 'x')
lpy, lvy, lcy, axvy, axcy = create_motion_plot(axes1[1], 'y')
lpz, lvz, lcz, axvz, axcz = create_motion_plot(axes1[2], 'z')

# ─────────────────────────
# 创建图2（Swing）
# ─────────────────────────
lswx, lawx, axawx = create_swing_plot(axes2[0], 'x')
lswy, lawy, axawy = create_swing_plot(axes2[1], 'y')


# ─────────────────────────
# 主循环
# ─────────────────────────
while plt.get_fignums():
    try:
        msg = sock.recv_json()
    except zmq.Again:
        plt.pause(0.05)
        continue

    ts = msg['timestamp']
    if t0 is None:
        t0 = ts
    buf['t'].append(ts - t0)

    for n in AXES:
        buf[f'pos_{n}'].append(msg['pos'][n])
        buf[f'vel_{n}'].append(msg['vel'][n])
        buf[f'cmd_{n}'].append(msg['cmd_vel'][n])

    buf['sw_x'].append(msg['swing']['swing_x'])
    buf['sw_y'].append(msg['swing']['swing_y'])
    buf['aw_x'].append(msg['swing']['ang_vel_x'])
    buf['aw_y'].append(msg['swing']['ang_vel_y'])

    t = list(buf['t'])

    # XYZ
    lpx.set_data(t, list(buf['pos_x']))
    lvx.set_data(t, list(buf['vel_x']))
    lcx.set_data(t, list(buf['cmd_x']))

    lpy.set_data(t, list(buf['pos_y']))
    lvy.set_data(t, list(buf['vel_y']))
    lcy.set_data(t, list(buf['cmd_y']))

    lpz.set_data(t, list(buf['pos_z']))
    lvz.set_data(t, list(buf['vel_z']))
    lcz.set_data(t, list(buf['cmd_z']))

    # Swing
    lswx.set_data(t, list(buf['sw_x']))
    lawx.set_data(t, list(buf['aw_x']))

    lswy.set_data(t, list(buf['sw_y']))
    lawy.set_data(t, list(buf['aw_y']))

    # 自动缩放
    for ax in axes1:
        ax.relim()
        ax.autoscale_view()

    for ax in axes2:
        ax.relim()
        ax.autoscale_view()

    for ax in [axvx, axcx, axvy, axcy, axvz, axcz, axawx, axawy]:
        ax.relim()
        ax.autoscale_view()

    plt.pause(0.02)