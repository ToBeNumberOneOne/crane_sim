"""Command-line interface for crane control.

This module provides an interactive CLI for controlling the crane
and monitoring its state in real-time.
"""

import threading
import mujoco


class CLI:
    """Interactive command-line interface for crane control."""

    def __init__(self, axes, state, model):
        """Initialize the CLI.

        Args:
            axes: Dictionary of AxisController instances (direct access)
            state: CraneState for monitoring (optional)
            model: MujocoModel for reset functionality
        """
        self.axes = axes
        self.state = state
        self.model = model
        self._quit_flag = False
        self._quit_lock = threading.Lock()

        width_list = [10, 12, 20]
        self.help_info = "\n --------- HELP INFO --------- "
        self.help_info += "\n {:<{}}".format("cmd", width_list[0])
        self.help_info += "{:^{}}".format("simple cmd", width_list[1])
        self.help_info += "{:<{}}".format("describ", width_list[2])

        self.cmd_list = []
        self.cmd_list.append(["h", "help", "帮助信息", self._help])
        self.cmd_list.append(["v", "velocity", "设置速度 v vx vy vz", self._velocity])
        self.cmd_list.append(["vx", "velocity_x", "设置x轴速度 vx value", lambda args: self._set_velocity(args, "x")])
        self.cmd_list.append(["vy", "velocity_y", "设置y轴速度 vy value", lambda args: self._set_velocity(args, "y")])
        self.cmd_list.append(["vz", "velocity_z", "设置z轴速度 vz value", lambda args: self._set_velocity(args, "z")])
        self.cmd_list.append(["1", "get_state", "获取当前状态", self._get_state])
        self.cmd_list.append(["s", "stop", "停止", self._stop])
        self.cmd_list.append(["r", "reset", "重置仿真", self._reset])
        self.cmd_list.append(["q", "quit", "退出程序", self._quit])

        for it in self.cmd_list:
            self.help_info += "\n {:<{}}".format(it[1], width_list[0])
            self.help_info += "{:^{}}".format(it[0], width_list[1])
            self.help_info += "{:<{}}".format(it[2], width_list[2])

    def is_quit_requested(self):
        """Check if quit was requested."""
        with self._quit_lock:
            return self._quit_flag

    def _help(self, args=None):
        """Display help information."""
        print(self.help_info)

    def run(self):
        """Run the interactive CLI loop."""
        self._help()
        while not self.is_quit_requested():
            try:
                cmd_line = input("please input cmd: ").strip()
                if not cmd_line:
                    continue
                parts = cmd_line.split()
                cmd = parts[0]
                args = parts[1:]
                found = False
                for it in self.cmd_list:
                    if it[0] == cmd or it[1] == cmd:
                        it[3](args)
                        found = True
                        break
                if not found:
                    print("未知命令，输入 h 或 help 查看帮助")
            except EOFError:
                # Handle Ctrl+D gracefully
                print("\n退出CLI")
                break
            except KeyboardInterrupt:
                # Handle Ctrl+C gracefully
                print("\n中断，输入 q 或 quit 退出程序")

    def _velocity(self, args):
        """Set velocities for all axes.

        Args:
            args: List of velocity values [vx, vy, vz]
        """
        if len(args) != 3:
            print("用法: v vx vy vz")
            return
        try:
            vx, vy, vz = map(float, args)
            # Direct access to axes
            self.axes['x'].set_velocity(vx)
            self.axes['y'].set_velocity(vy)
            self.axes['z'].set_velocity(vz)
            print(f"设置速度: vx={vx}, vy={vy}, vz={vz}")
        except ValueError:
            print("参数错误，用法: v vx vy vz")
        except KeyError as e:
            print(f"未知轴: {e}")
        except Exception as e:
            print(f"设置速度失败: {e}")

    def _set_velocity(self, args, axis):
        """Set velocity for a single axis.

        Args:
            args: List containing single velocity value
            axis: Axis name ('x', 'y', or 'z')
        """
        if len(args) != 1:
            print(f"用法: v{axis} value")
            return
        try:
            value = float(args[0])
            # Direct access to axis
            self.axes[axis].set_velocity(value)
            print(f"设置{axis}轴速度: v{axis}={value}")
        except ValueError:
            print(f"参数错误，用法: v{axis} value")
        except KeyError:
            print(f"未知轴: {axis}")
        except Exception as e:
            print(f"设置速度失败: {e}")

    def _get_state(self, args=None):
        """Display current crane state."""
        if self.state is None:
            print("未找到状态对象")
            return

        state = self.state.get_latest()
        if state is None:
            print("尚未采集状态数据")
            return

        def fmt(val):
            return f"{float(val):.3f}" if isinstance(val, (int, float)) else str(val)

        print("\n  当前起重机状态")
        print("  {:<8} {:<8} {:<8} {:<8}".format("物理量", "x轴", "y轴", "z轴"))
        print("  {:<8} {:<8} {:<8} {:<8}".format(
            "位置(m)",
            fmt(state['pos'].get('x', 'N/A')),
            fmt(state['pos'].get('y', 'N/A')),
            fmt(state['pos'].get('z', 'N/A'))
        ))
        print("  {:<8} {:<8} {:<8} {:<8}".format(
            "速度(m/s)",
            fmt(state['vel'].get('x', 'N/A')),
            fmt(state['vel'].get('y', 'N/A')),
            fmt(state['vel'].get('z', 'N/A'))
        ))
        print("  {:<8} {:<8} {:<8} {:<8}".format(
            "指令(m/s)",
            fmt(state['cmd_vel'].get('x', 'N/A')),
            fmt(state['cmd_vel'].get('y', 'N/A')),
            fmt(state['cmd_vel'].get('z', 'N/A'))
        ))
        print()

    def _stop(self, args=None):
        """Stop all axes."""
        for axis in self.axes.values():
            axis.set_velocity(0.0)
        print("停止运动")

    def _reset(self, args=None):
        """Reset simulation state."""
        # Reset MuJoCo physics state
        mujoco.mj_resetData(self.model.model, self.model.data)

        # Reset all axis controllers
        for axis in self.axes.values():
            axis.reset()

        print("仿真状态已重置")

    def _quit(self, args=None):
        """Request simulation to quit."""
        with self._quit_lock:
            self._quit_flag = True
        print("退出信号已发送")