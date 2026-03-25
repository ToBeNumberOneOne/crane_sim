"""PLC S7 controller for crane control.

This module provides Siemens S7 PLC control interface using snap7."""

import time
import threading
from typing import Optional
from crane_sim.cli.cli import CLI

try:
    import snap7
    from snap7.util import get_real, set_real, get_bool, set_bool
    SNAP7_AVAILABLE = True
except ImportError:
    SNAP7_AVAILABLE = False
    print("警告: snap7 未安装，PLC功能不可用")
    print("安装方法: pip install python-snap7")


class PLCController:
    """S7 PLC controller for crane.

    Reads velocity commands from PLC and writes back position/velocity feedback.
    """

    def __init__(self, axes, state, manager, plc_ip: str, db_number: int = 1):
        """Initialize PLC controller.

        Args:
            axes: Dictionary of AxisController instances
            state: CraneState instance
            manager: ControllerManager instance
            plc_ip: PLC IP address
            db_number: DB block number (default 1)
        """
        self.axes = axes
        self.state = state
        self.manager = manager
        self.plc_ip = plc_ip
        self.rack = 0
        self.slot = 1
        self.db_number = db_number

        self._running = False
        self._thread = None
        self.client: Optional['snap7.client.Client'] = None
        self.connected = False
        self._reconnect_interval = 5.0  # seconds between reconnect attempts

        # DB offsets
        self.OFFSET_POS_X = 0
        self.OFFSET_POS_Y = 4
        self.OFFSET_POS_Z = 8
        self.OFFSET_VEL_X = 12
        self.OFFSET_VEL_Y = 16
        self.OFFSET_VEL_Z = 20
        self.OFFSET_SWING_X = 24
        self.OFFSET_SWING_Y = 28
        self.OFFSET_ANG_VEL_X = 32
        self.OFFSET_ANG_VEL_Y = 36
        self.OFFSET_ANG_VEL_Z = 40

        self.OFFSET_CMD_VX = 100
        self.OFFSET_CMD_VY = 104
        self.OFFSET_CMD_VZ = 108
        self.OFFSET_ESTOP = 112
        self.OFFSET_RESET = 112

        self.DB_SIZE = 116  # Total size in bytes

        # Update rate
        self.update_interval = 0.05  # 20Hz

    def is_available(self) -> bool:
        """Check if snap7 library is available."""
        return SNAP7_AVAILABLE

    def connect(self) -> bool:
        """Connect to PLC.

        Returns:
            True if connection successful
        """
        if not self.is_available():
            print("PLC: snap7库未安装")
            return False

        try:
            self.client = snap7.client.Client()
            self.client.connect(self.plc_ip, self.rack, self.slot)
            self.connected = True
            print(f"  - PLC已连接: {self.plc_ip} (DB{self.db_number})")
            return True
        except Exception as e:
            print(f"  - PLC连接失败: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from PLC."""
        if self.client and self.connected:
            self.client.disconnect()
            self.connected = False
            print("PLC已断开连接")

    def _read_commands(self) -> tuple:
        """Read velocity commands from PLC.

        Returns:
            Tuple of (vx, vy, vz, estop, reset)

        Raises:
            Exception: on communication failure (caller triggers reconnect)
        """
        db_data = self.client.db_read(self.db_number, 0, self.DB_SIZE)

        vx = get_real(db_data, self.OFFSET_CMD_VX)
        vy = get_real(db_data, self.OFFSET_CMD_VY)
        vz = get_real(db_data, self.OFFSET_CMD_VZ)

        estop = get_bool(db_data, self.OFFSET_ESTOP, 0)
        reset = get_bool(db_data, self.OFFSET_RESET, 1)

        return vx, vy, vz, estop, reset

    def _write_feedback(self):
        """Write position and velocity feedback to PLC.

        Raises:
            Exception: on communication failure (caller triggers reconnect)
        """
        state = self.state.get_latest()
        if state is None:
            return

        db_data = bytearray(100)

        set_real(db_data, self.OFFSET_POS_X, state['pos'].get('x', 0.0))
        set_real(db_data, self.OFFSET_POS_Y, state['pos'].get('y', 0.0))
        set_real(db_data, self.OFFSET_POS_Z, state['pos'].get('z', 0.0))

        set_real(db_data, self.OFFSET_VEL_X, state['vel'].get('x', 0.0))
        set_real(db_data, self.OFFSET_VEL_Y, state['vel'].get('y', 0.0))
        set_real(db_data, self.OFFSET_VEL_Z, state['vel'].get('z', 0.0))

        swing_data = state.get('swing', {})
        set_real(db_data, self.OFFSET_SWING_X, swing_data.get('swing_x', 0.0))
        set_real(db_data, self.OFFSET_SWING_Y, swing_data.get('swing_y', 0.0))
        set_real(db_data, self.OFFSET_ANG_VEL_X, swing_data.get('ang_vel_x', 0.0))
        set_real(db_data, self.OFFSET_ANG_VEL_Y, swing_data.get('ang_vel_y', 0.0))
        set_real(db_data, self.OFFSET_ANG_VEL_Z, swing_data.get('ang_vel_z', 0.0))

        self.client.db_write(self.db_number, 0, db_data)

    def reset(self):
        """Reset simulation to home position using shared CLI method."""
        print("PLC: 重置仿真")
        CLI.perform_reset(self.manager.model, self.axes)

    def run(self):
        """Main control loop (runs in separate thread).

        Handles initial connection and automatic reconnection on failure.
        """
        self._running = True
        print("PLC控制器已启动，等待连接...")

        last_reset = False

        while self._running:
            # ── 连接阶段：未连接时持续重试 ──────────────────────────────
            if not self.connected:
                if not self.connect():
                    # 等待重试，期间每秒检查一次 _running 标志
                    for _ in range(int(self._reconnect_interval)):
                        if not self._running:
                            break
                        time.sleep(1.0)
                    continue
                print("PLC控制器已就绪")

            # ── 正常工作阶段 ─────────────────────────────────────────────
            try:
                if self.manager.is_active("plc"):
                    vx, vy, vz, estop, reset = self._read_commands()

                    if estop:
                        for axis in self.axes.values():
                            axis.set_velocity(0.0)
                    else:
                        self.axes['x'].set_velocity(vx)
                        self.axes['y'].set_velocity(vy)
                        self.axes['z'].set_velocity(vz)

                    if reset and not last_reset:
                        self.reset()
                    last_reset = reset

                # Always write feedback (even if not active mode)
                self._write_feedback()

                time.sleep(self.update_interval)

            except Exception as e:
                print(f"PLC通信错误: {e}，{self._reconnect_interval:.0f}秒后尝试重连...")
                self.connected = False
                # 停轴，避免失控
                for axis in self.axes.values():
                    axis.set_velocity(0.0)

        self.disconnect()
        print("PLC控制器已停止")

    def start(self):
        """Start controller in background thread."""
        if not self._running:
            self._thread = threading.Thread(target=self.run, daemon=True)
            self._thread.start()

    def stop(self):
        """Stop the controller."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
