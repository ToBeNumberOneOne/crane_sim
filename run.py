"""Main entry point for crane simulation."""

import threading

import mujoco.viewer

from crane_sim.config.config import CraneConfig
from crane_sim.core.mujoco_model import MujocoModel
from crane_sim.core.axis_controller import AxisController
from crane_sim.core.joint_mapper import JointMapper
from crane_sim.core.state import CraneState
from crane_sim.cli.cli import CLI
from crane_sim.runtime.sim_loop import SimulationLoop


def main():
    """Initialize and run the crane simulation."""
    print("=" * 60)
    print("桥式起重机仿真系统 (Bridge Crane Simulator)")
    print("=" * 60)

    # 1. 加载仿真和控制配置
    print("\n[1/7] 加载配置文件...")
    cfg = CraneConfig("config/crane.yaml")
    print(f"  - 仿真步长: {cfg.sim.timestep}s")
    print(f"  - 数据发布频率: {cfg.publish_rate_hz}Hz")
    print(f"  - 配置轴数: {len(cfg.axes)}")

    # 2. 初始化物理模型（MuJoCo）
    print("\n[2/7] 初始化物理模型...")
    model = MujocoModel("models/bridge_crane.xml", cfg.sim.timestep)
    print(f"  - 模型: bridge_crane.xml")
    print(f"  - 自由度数: {model.model.nv}")

    # 3. 创建 Joint 映射器（修复索引问题）
    print("\n[3/7] 创建 Joint 映射器...")
    mapper = JointMapper(model.model, cfg.axes)

    # 4. 初始化各轴控制器（x/y/z），线程安全
    print("\n[4/7] 初始化轴控制器...")
    axes = {
        name: AxisController(model.model, axis_cfg, name)
        for name, axis_cfg in cfg.axes.items()
    }
    print(f"  - 已初始化 {len(axes)} 个轴控制器（线程安全）")

    # 5. 初始化状态采集与发布模块
    print("\n[5/7] 初始化状态监控...")
    state = CraneState()
    state.start_publish(interval=cfg.publish_interval)
    print(f"  - ZMQ 发布端口: tcp://*:5555")
    print(f"  - 发布间隔: {cfg.publish_interval:.3f}s")

    # 6. 启动MuJoCo可视化窗口
    print("\n[6/7] 启动可视化窗口...")
    viewer = mujoco.viewer.launch_passive(model.model, model.data)
    print("  - MuJoCo 可视化已启动")

    # 7. 启动命令行交互线程（CLI），直接操作 axes
    print("\n[7/7] 启动命令行交互...")
    cli = CLI(axes, state, model)
    threading.Thread(target=cli.run, daemon=True).start()
    print("  - CLI 线程已启动（直接控制轴）")

    # 8. 启动仿真主循环，实时同步物理仿真、控制和状态
    print("\n" + "=" * 60)
    print("系统就绪，开始仿真...")
    print("=" * 60 + "\n")

    try:
        SimulationLoop(model, axes, state, mapper, cli).run(viewer)
    except KeyboardInterrupt:
        print("\n\n仿真被用户中断")
    finally:
        print("\n清理资源...")
        state.stop_publish()
        print("仿真已结束")


if __name__ == "__main__":
    main()
