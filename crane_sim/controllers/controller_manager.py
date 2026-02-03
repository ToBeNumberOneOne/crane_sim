"""Controller manager for handling multiple control modes.

This module manages switching between different control modes:
CLI, Xbox gamepad, and PLC control.
"""

import threading
from typing import Dict, Optional


class ControllerManager:
    """Manages multiple control modes and switching between them.

    Only the active controller can operate the axes at any time,
    preventing conflicts between different control sources.
    """

    def __init__(self, axes, state, model):
        """Initialize the controller manager.

        Args:
            axes: Dictionary of AxisController instances
            state: CraneState instance
            model: MujocoModel instance
        """
        self.axes = axes
        self.state = state
        self.model = model
        self.current_mode = "cli"
        self._mode_lock = threading.Lock()

        # Controllers will be registered here
        self.controllers: Dict[str, Optional[object]] = {
            "cli": None,  # CLI is always active in background
            "xbox": None,  # Will be initialized if gamepad available
            "plc": None,   # Will be initialized if PLC configured
        }

    def register_controller(self, name: str, controller):
        """Register a controller.

        Args:
            name: Controller name (cli, xbox, plc)
            controller: Controller instance
        """
        with self._mode_lock:
            self.controllers[name] = controller

    def set_mode(self, mode: str) -> bool:
        """Switch control mode.

        Args:
            mode: Target mode name

        Returns:
            True if switch successful, False otherwise
        """
        with self._mode_lock:
            if mode not in self.controllers:
                return False

            if self.controllers[mode] is None:
                return False

            old_mode = self.current_mode
            self.current_mode = mode
            print(f"控制模式已切换: {old_mode} → {mode}")
            return True

    def get_mode(self) -> str:
        """Get current control mode.

        Returns:
            Current mode name
        """
        with self._mode_lock:
            return self.current_mode

    def is_active(self, mode: str) -> bool:
        """Check if a specific mode is active.

        Args:
            mode: Mode name to check

        Returns:
            True if the mode is currently active
        """
        with self._mode_lock:
            return self.current_mode == mode

    def list_available_modes(self) -> list:
        """List all available control modes.

        Returns:
            List of available mode names
        """
        with self._mode_lock:
            return [name for name, ctrl in self.controllers.items() if ctrl is not None]
