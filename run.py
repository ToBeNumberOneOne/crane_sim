import threading

import mujoco.viewer

from crane_sim.config.config import CraneConfig,ControlContext
from crane_sim.core.mujoco_model import MujocoModel
from crane_sim.core.axis_controller import AxisController
from crane_sim.core.state import CraneState
from crane_sim.cli.cli import CLI 
from crane_sim.runtime.sim_loop import SimulationLoop



def main():
    # 1. 加载仿真和控制配置
    cfg = CraneConfig("config/crane.yaml")

    # 2. 初始化物理模型（MuJoCo）
    model = MujocoModel("models/bridge_crane.xml", cfg.sim.timestep)
 
 

    # 3. 初始化各轴控制器（x/y/z）
    axes = {
        name: AxisController(model.model, axis_cfg)
        for name, axis_cfg in cfg.axes.items()
    }

    # 4. 初始化状态采集与发布模块
    state = CraneState()
    state.start_publish(interval=0.1)  # 定时发布状态，供外部订阅

    # 5. 启动MuJoCo可视化窗口
    viewer = mujoco.viewer.launch_passive(model.model, model.data)

    # 6. 初始化控制上下文（保存控制指令）
    ctrl = ControlContext()

    # 7. 启动命令行交互线程（CLI），支持实时输入控制指令
    cli = CLI(ctrl, state)
    threading.Thread(target=cli.run, daemon=True).start()

    # 8. 启动仿真主循环，实时同步物理仿真、控制和状态
    SimulationLoop(model, axes, state, ctrl).run(viewer)

if __name__ == "__main__":
    main()