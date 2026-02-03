"""Xbox gamepad controller for crane control.

This module provides Xbox gamepad control interface:
- Left stick X: X-axis velocity (crane left/right)
- Left stick Y: Y-axis velocity (trolley forward/backward)
- Right stick Y: Z-axis velocity (hoist up/down)
- A button: Emergency stop
- B button: Reset simulation
"""

import pygame
import time
import threading


class XboxController:
    """Xbox gamepad controller for crane.

    Maps gamepad axes to crane velocities with deadzone filtering
    and velocity scaling.
    """

    def __init__(self, axes, state, manager):
        """Initialize Xbox controller.

        Args:
            axes: Dictionary of AxisController instances
            state: CraneState instance
            manager: ControllerManager instance
        """
        self.axes = axes
        self.state = state
        self.manager = manager
        self._running = False
        self._thread = None

        # Controller settings
        self.deadzone = 0.15  # Ignore small stick movements
        self.max_speeds = {
            'x': 2.0,  # m/s
            'y': 1.0,  # m/s
            'z': 2.0,  # m/s
        }

        # Initialize pygame
        pygame.init()
        pygame.joystick.init()

        # Force refresh (critical for detection)
        pygame.joystick.quit()
        pygame.joystick.init()

        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"  - Xbox手柄已连接: {self.joystick.get_name()}")
        else:
            print("  - 未检测到Xbox手柄")

        self.last_button_state = {}

    def update(self):
        """Update controller state (called from main simulation loop)."""
        if not self.is_available():
            return

        # Process pygame events
        pygame.event.pump()

        # Only control if this mode is active
        if not self.manager.is_active("xbox"):
            return

        # Read stick axes
        left_x = self.joystick.get_axis(0)
        left_y = self.joystick.get_axis(1)
        right_y = self.joystick.get_axis(3)

        # Apply deadzone
        left_x = self._apply_deadzone(left_x)
        left_y = self._apply_deadzone(left_y)
        right_y = self._apply_deadzone(right_y)

        # Map to velocities
        vx = left_x * self.max_speeds['x']
        vy = -left_y * self.max_speeds['y']
        vz = -right_y * self.max_speeds['z']

        # Set velocities
        self.axes['x'].set_velocity(vx)
        self.axes['y'].set_velocity(vy)
        self.axes['z'].set_velocity(vz)

        # Check buttons
        if self.joystick.get_button(0):
            if not self.last_button_state.get(0, False):
                print("Xbox: 紧急停止")
                for axis in self.axes.values():
                    axis.set_velocity(0.0)
            self.last_button_state[0] = True
        else:
            self.last_button_state[0] = False

        if self.joystick.get_button(1):
            if not self.last_button_state.get(1, False):
                print("Xbox: 重置仿真")
                import mujoco
                mujoco.mj_resetData(self.manager.model.model, self.manager.model.data)
                for axis in self.axes.values():
                    axis.reset()
            self.last_button_state[1] = True
        else:
            self.last_button_state[1] = False

    def is_available(self) -> bool:
        """Check if gamepad is connected.

        Returns:
            True if gamepad is available
        """
        return self.joystick is not None

    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to stick input.

        Args:
            value: Raw stick value (-1 to 1)

        Returns:
            Filtered value with deadzone applied
        """
        if abs(value) < self.deadzone:
            return 0.0
        # Scale to full range after deadzone
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def run(self):
        """Main control loop - now handled by update() in main thread."""
        pass

    def start(self):
        """Start controller - no background thread needed."""
        if self.is_available():
            print("Xbox手柄控制器已启动")

    def stop(self):
        """Stop the controller."""
        pass


