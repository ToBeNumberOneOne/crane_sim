"""PLC S7 controller for crane control.

This module provides Siemens S7 PLC control interface using snap7.

DB Block Layout (example using DB1):
  DBD0  (REAL) - X-axis position feedback (m)
  DBD4  (REAL) - Y-axis position feedback (m)
  DBD8  (REAL) - Z-axis position feedback (m)
  DBD12 (REAL) - X-axis velocity feedback (m/s)
  DBD16 (REAL) - Y-axis velocity feedback (m/s)
  DBD20 (REAL) - Z-axis velocity feedback (m/s)
  DBD24 (REAL) - X-axis velocity command (m/s)
  DBD28 (REAL) - Y-axis velocity command (m/s)
  DBD32 (REAL) - Z-axis velocity command (m/s)
  DBX36.0 (BOOL) - Emergency stop signal
  DBX36.1 (BOOL) - Reset signal
  DBX37.0 (BOOL) - Connection status (write by PLC)
  DBD40 (REAL) - Swing angle X feedback (rad)
  DBD44 (REAL) - Swing angle Y feedback (rad)
  DBD48 (REAL) - Angular velocity X feedback (rad/s)
  DBD52 (REAL) - Angular velocity Y feedback (rad/s)
  DBD56 (REAL) - Angular velocity Z feedback (rad/s)
"""

import struct
import time
import threading
from typing import Optional

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

    def __init__(self, axes, state, manager, plc_ip: str, rack: int = 0, slot: int = 1, db_number: int = 1):
        """Initialize PLC controller.

        Args:
            axes: Dictionary of AxisController instances
            state: CraneState instance
            manager: ControllerManager instance
            plc_ip: PLC IP address
            rack: PLC rack number (default 0)
            slot: PLC slot number (default 1)
            db_number: DB block number (default 1)
        """
        self.axes = axes
        self.state = state
        self.manager = manager
        self.plc_ip = plc_ip
        self.rack = rack
        self.slot = slot
        self.db_number = db_number

        self._running = False
        self._thread = None
        self.client: Optional['snap7.client.Client'] = None
        self.connected = False

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

        self.DB_SIZE = 60  # Total size in bytes

        # Update rate
        self.update_interval = 0.05  # 20Hz

    def is_available(self) -> bool:
        """Check if snap7 library is available.

        Returns:
            True if snap7 is installed
        """
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
        """
        try:
            # Read entire DB block
            db_data = self.client.db_read(self.db_number, 0, self.DB_SIZE)

            # Parse velocity commands
            vx = get_real(db_data, self.OFFSET_CMD_VX)
            vy = get_real(db_data, self.OFFSET_CMD_VY)
            vz = get_real(db_data, self.OFFSET_CMD_VZ)

            # Parse control signals
            estop = get_bool(db_data, self.OFFSET_ESTOP, 0)
            reset = get_bool(db_data, self.OFFSET_RESET, 1)

            return vx, vy, vz, estop, reset

        except Exception as e:
            print(f"PLC读取错误: {e}")
            return 0.0, 0.0, 0.0, False, False

    def _write_feedback(self):
        """Write position and velocity feedback to PLC."""
        try:
            # Get latest state
            state = self.state.get_latest()
            if state is None:
                return

            # Prepare data buffer
            db_data = bytearray(self.DB_SIZE)

            # Write position feedback
            set_real(db_data, self.OFFSET_POS_X, state['pos'].get('x', 0.0))
            set_real(db_data, self.OFFSET_POS_Y, state['pos'].get('y', 0.0))
            set_real(db_data, self.OFFSET_POS_Z, state['pos'].get('z', 0.0))

            # Write velocity feedback
            set_real(db_data, self.OFFSET_VEL_X, state['vel'].get('x', 0.0))
            set_real(db_data, self.OFFSET_VEL_Y, state['vel'].get('y', 0.0))
            set_real(db_data, self.OFFSET_VEL_Z, state['vel'].get('z', 0.0))

            # Write swing data feedback
            swing_data = state.get('swing', {})
            set_real(db_data, self.OFFSET_SWING_X, swing_data.get('swing_x', 0.0))
            set_real(db_data, self.OFFSET_SWING_Y, swing_data.get('swing_y', 0.0))
            set_real(db_data, self.OFFSET_ANG_VEL_X, swing_data.get('ang_vel_x', 0.0))
            set_real(db_data, self.OFFSET_ANG_VEL_Y, swing_data.get('ang_vel_y', 0.0))
            set_real(db_data, self.OFFSET_ANG_VEL_Z, swing_data.get('ang_vel_z', 0.0))

            # Write to PLC
            self.client.db_write(self.db_number, 0, db_data)

        except Exception as e:
            print(f"PLC写入错误: {e}")

    def run(self):
        """Main control loop (runs in separate thread)."""
        if not self.connect():
            return

        self._running = True
        print("PLC控制器已启动")

        last_reset = False

        while self._running:
            try:
                # Only control if this mode is active
                if self.manager.is_active("plc"):
                    # Read commands from PLC
                    vx, vy, vz, estop, reset = self._read_commands()

                    # Handle emergency stop
                    if estop:
                        for axis in self.axes.values():
                            axis.set_velocity(0.0)
                    else:
                        # Set velocities (thread-safe)
                        self.axes['x'].set_velocity(vx)
                        self.axes['y'].set_velocity(vy)
                        self.axes['z'].set_velocity(vz)

                    # Handle reset signal (edge triggered)
                    if reset and not last_reset:
                        print("PLC: 重置仿真")
                        import mujoco
                        mujoco.mj_resetData(self.manager.model.model, self.manager.model.data)
                        for axis in self.axes.values():
                            axis.reset()
                    last_reset = reset

                # Always write feedback (even if not active mode)
                self._write_feedback()

                # Sleep
                time.sleep(self.update_interval)

            except Exception as e:
                print(f"PLC控制器错误: {e}")
                time.sleep(0.5)

        self.disconnect()
        print("PLC控制器已停止")

    def start(self):
        """Start controller in background thread."""
        if self.is_available() and not self._running:
            self._thread = threading.Thread(target=self.run, daemon=True)
            self._thread.start()

    def stop(self):
        """Stop the controller."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
