"""Axis controller for crane control.

This module provides thread-safe control for individual crane axes.
"""

import mujoco
import threading


class AxisController:
    """Controller for a single crane axis (x, y, or z).

    Handles velocity limiting, acceleration limiting, and soft limits.
    Thread-safe for concurrent access from CLI and simulation loop.
    """

    def __init__(self, model, axis_cfg, axis_name):
        """Initialize the axis controller.

        Args:
            model: MuJoCo model
            axis_cfg: AxisConfig object containing parameters
            axis_name: Name of this axis (e.g., 'x', 'y', 'z')
        """
        self.id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, axis_cfg.actuator
        )
        self.cfg = axis_cfg
        self.axis_name = axis_name
        self._lock = threading.Lock()
        self.cmd_v = 0.0
        self.cur_v = 0.0

    def set_velocity(self, v):
        """Set the commanded velocity (thread-safe).

        Args:
            v: Desired velocity in m/s (will be clamped to vmax)
        """
        with self._lock:
            self.cmd_v = max(
                -self.cfg.vmax, min(self.cfg.vmax, v)
            )

    def reset(self):
        """Reset controller state to zero (thread-safe)."""
        with self._lock:
            self.cmd_v = 0.0
            self.cur_v = 0.0

    def get_current_velocity(self):
        """Get the current output velocity (thread-safe).

        Returns:
            Current velocity being sent to actuator
        """
        with self._lock:
            return self.cur_v

    def update(self, dt, mapper, data):
        """Update the controller and return the control output.

        Args:
            dt: Time step in seconds
            mapper: JointMapper instance for accessing joint positions
            data: MuJoCo data object

        Returns:
            Current velocity command to be sent to actuator
        """
        with self._lock:
            # Get current position using mapper (without offset for limit checking)
            pos = mapper.get_position(data, self.axis_name)

            # Soft limit checking
            if pos < self.cfg.soft_limit[0] and self.cmd_v < 0:
                self.cmd_v = 0
            if pos > self.cfg.soft_limit[1] and self.cmd_v > 0:
                self.cmd_v = 0

            # Acceleration limiting
            dv = self.cfg.amax * dt
            self.cur_v += max(-dv, min(dv, self.cmd_v - self.cur_v))

            return self.cur_v
