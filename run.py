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
from crane_sim.controllers import ControllerManager
from crane_sim.controllers.xbox_controller import XboxController
from crane_sim.controllers.plc_controller import PLCController


def main():
    """Initialize and run the crane simulation."""
    print("=" * 60)
    print("桥式起重机仿真系统 (Bridge Crane Simulator)")
    print("=" * 60)

    # 1. 加载仿真和控制配置
    print("\n[1/8] 加载配置文件...")
    cfg = CraneConfig("config/crane.yaml")
    print(f"  - 仿真步长: {cfg.sim.timestep}s")
    print(f"  - 数据发布频率: {cfg.publish_rate_hz}Hz")
    print(f"  - 配置轴数: {len(cfg.axes)}")

    # 2. 初始化物理模型（MuJoCo）
    print("\n[2/8] 初始化物理模型...")
    model = MujocoModel("models/bridge_crane.xml", cfg.sim.timestep)
    print(f"  - 模型: bridge_crane.xml")
    print(f"  - 自由度数: {model.model.nv}")

    # 3. 创建 Joint 映射器（修复索引问题）
    print("\n[3/8] 创建 Joint 映射器...")
    mapper = JointMapper(model.model, cfg.axes)

    # 4. 初始化各轴控制器（x/y/z），线程安全
    print("\n[4/8] 初始化轴控制器...")
    axes = {
        name: AxisController(model.model, axis_cfg, name)
        for name, axis_cfg in cfg.axes.items()
    }
    print(f"  - 已初始化 {len(axes)} 个轴控制器（线程安全）")

    # 5. 初始化状态采集与发布模块
    print("\n[5/8] 初始化状态监控...")
    state = CraneState()
    state.start_publish(interval=cfg.publish_interval)
    print(f"  - ZMQ 发布端口: tcp://*:5555")
    print(f"  - 发布间隔: {cfg.publish_interval:.3f}s")

    # 6. 创建控制器管理器
    print("\n[6/8] 初始化控制器...")
    manager = ControllerManager(axes, state, model)

    # 6a. 初始化并启动Xbox手柄控制器
    xbox = XboxController(axes, state, manager)
    if xbox.is_available():
        manager.register_controller("xbox", xbox)
        xbox.start()
    else:
        print("  - Xbox手柄不可用")

    # 6b. 初始化PLC控制器
    
    plc = PLCController(axes, state, manager, cfg.sim.plc_ip, cfg.sim.db_number)
    if plc.is_available():
        manager.register_controller("plc", plc)
        plc.start()

    # 6c. 注册CLI控制器
    manager.register_controller("cli", "active")  # CLI always available

    # 7. 启动MuJoCo可视化窗口
    print("\n[7/8] 启动可视化窗口...")
    viewer = mujoco.viewer.launch_passive(model.model, model.data)
    print("  - MuJoCo 可视化已启动")

    # 8. 启动命令行交互线程（CLI），支持模式切换
    print("\n[8/8] 启动命令行交互...")
    cli = CLI(axes, state, model, manager)
    threading.Thread(target=cli.run, daemon=True).start()
    print("  - CLI 线程已启动（支持模式切换）")

    # 9. 启动仿真主循环
    print("\n" + "=" * 60)
    print("系统就绪，开始仿真...")
    print(f"当前控制模式: {manager.get_mode()}")
    print("输入 'h' 查看帮助，'m' 切换控制模式")
    print("=" * 60 + "\n")

    try:
        SimulationLoop(model, axes, state, mapper, cli).run(viewer)
    except KeyboardInterrupt:
        print("\n\n仿真被用户中断")
    finally:
        print("\n清理资源...")
        state.stop_publish()
        if xbox.is_available():
            xbox.stop()
        print("仿真已结束")


if __name__ == "__main__":
    main()
