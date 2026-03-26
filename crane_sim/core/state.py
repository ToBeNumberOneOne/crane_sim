"""Crane state monitoring and publishing.

This module provides state monitoring and ZMQ-based
state publishing for external monitoring tools.
"""

import zmq
import threading
import time
from typing import Dict, Optional
import mujoco


class CraneState:
    """Crane state monitoring and publishing system.

    Collects state snapshots and publishes them via ZMQ for external monitoring.
    """

    def __init__(self):
        """Initialize the state monitoring system."""
        self._latest_state: Optional[Dict] = None
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._publish_interval: float = 0.02  # default 50Hz
        self._last_publish_time: float = 0.0
        self.plc = None  # Will be set later by set_plc_controller()

        # ZMQ publisher
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")

    def sample(self, mapper, data: mujoco.MjData, axes) -> Dict:
        """Create a state snapshot from current simulation state.

        Args:
            mapper: JointMapper for accessing joint positions/velocities
            data: MuJoCo data object
            axes: Dictionary of AxisController instances

        Returns:
            Dictionary with current state
        """
        # Collect commanded velocities from each axis controller
        cmd_velocities = {
            name: axis.get_current_velocity()
            for name, axis in axes.items()
        }

        state = {
            "timestamp": time.time(),
            "pos": mapper.get_all_positions(data),
            "vel": mapper.get_all_velocities(data),
            "cmd_vel": cmd_velocities,
            "swing": mapper.get_swing_data(data)
        }

        # Add PLC control signals (always available regardless of control mode)
        if self.plc:
            state["control_signals"] = {
                "start": self.plc.plc_start,
                "running": self.plc.plc_running
            }

        # Store latest state (thread-safe)
        with self._state_lock:
            self._latest_state = state

        # Publish directly in sim loop thread — no inter-thread delay
        if state["timestamp"] - self._last_publish_time >= self._publish_interval:
            try:
                self.socket.send_json(state, zmq.NOBLOCK)
            except zmq.Again:
                pass  # subscriber not ready, drop this frame
            self._last_publish_time = state["timestamp"]

        return state

    def get_latest(self) -> Optional[Dict]:
        """Get the most recent state snapshot.

        Returns:
            Latest state dict, or None if no samples taken yet
        """
        with self._state_lock:
            return self._latest_state

    def set_plc_controller(self, plc):
        """Set the PLC controller for accessing control signals.

        Args:
            plc: PLCController instance
        """
        self.plc = plc

    def start_publish(self, interval=0.1):
        """Configure publish rate. Publishing happens directly in sample().

        Args:
            interval: Publishing interval in seconds
        """
        self._publish_interval = interval

    def stop_publish(self):
        """Stop the publishing thread."""
        self._stop_event.set()