"""Joint to qpos/qvel index mapper for MuJoCo model.

This module provides a unified interface to access joint positions and velocities
from MuJoCo's qpos/qvel arrays, handling complex joint types (ball joints, etc.)
and applying configuration offsets.
"""

import mujoco
import numpy as np
from typing import Dict, Tuple
from crane_sim.config.config import AxisConfig


class JointMapper:
    """Maps joint names to qpos/qvel indices and handles offset transformations."""

    def __init__(self, model: mujoco.MjModel, joint_configs: Dict[str, AxisConfig]):
        """Initialize the joint mapper.

        Args:
            model: MuJoCo model containing joint definitions
            joint_configs: Dictionary mapping joint names (x, y, z) to their configs
        """
        self.model = model
        self.joint_configs = joint_configs

        # Build mapping from joint name to qpos/qvel indices
        self._qpos_indices = {}
        self._qvel_indices = {}
        self._joint_ids = {}

        for axis_name, axis_cfg in joint_configs.items():
            # Get actuator ID
            actuator_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, axis_cfg.actuator
            )
            if actuator_id == -1:
                raise ValueError(f"Actuator '{axis_cfg.actuator}' not found")

            # Get joint ID from actuator
            joint_id = model.actuator_trnid[actuator_id, 0]
            if joint_id == -1:
                raise ValueError(f"No joint found for actuator '{axis_cfg.actuator}'")

            # Store indices
            self._qpos_indices[axis_name] = model.jnt_qposadr[joint_id]
            self._qvel_indices[axis_name] = model.jnt_dofadr[joint_id]
            self._joint_ids[axis_name] = joint_id

        # Find ball joint (rope_joint) for swing angle
        self._ball_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rope_joint")
        if self._ball_joint_id != -1:
            self._ball_qpos_addr = model.jnt_qposadr[self._ball_joint_id]
            self._ball_qvel_addr = model.jnt_dofadr[self._ball_joint_id]
        else:
            self._ball_joint_id = None
            self._ball_qpos_addr = None
            self._ball_qvel_addr = None

        # Log the mappings
        print("\n=== Joint Mapper Initialized ===")
        for axis_name in joint_configs.keys():
            joint_id = self._joint_ids[axis_name]
            qpos_idx = self._qpos_indices[axis_name]
            qvel_idx = self._qvel_indices[axis_name]
            joint_name = model.joint(joint_id).name
            print(f"  {axis_name} axis: joint '{joint_name}' (id={joint_id}) -> qpos[{qpos_idx}], qvel[{qvel_idx}]")

        if self._ball_joint_id is not None:
            print(f"  swing: joint 'rope_joint' (id={self._ball_joint_id}) -> qpos[{self._ball_qpos_addr}:+4], qvel[{self._ball_qvel_addr}:+3]")

        print("================================\n")

    def get_position(self, data: mujoco.MjData, axis_name: str) -> float:
        """Get the position of a specific axis, with offset applied."""
        qpos_idx = self._qpos_indices[axis_name]
        offset = self.joint_configs[axis_name].offset
        return round(float(data.qpos[qpos_idx] + offset),3)

    def get_velocity(self, data: mujoco.MjData, axis_name: str) -> float:
        """Get the velocity of a specific axis."""
        qvel_idx = self._qvel_indices[axis_name]
        return round(float(data.qvel[qvel_idx]),3)

    def get_all_positions(self, data: mujoco.MjData) -> Dict[str, float]:
        """Get positions of all configured axes."""
        return {
            axis_name: self.get_position(data, axis_name)
            for axis_name in self.joint_configs.keys()
        }

    def get_all_velocities(self, data: mujoco.MjData) -> Dict[str, float]:
        """Get velocities of all configured axes."""
        return {
            axis_name: self.get_velocity(data, axis_name)
            for axis_name in self.joint_configs.keys()
        }

    def get_raw_position(self, data: mujoco.MjData, axis_name: str) -> float:
        """Get the raw position without offset (for soft limits)."""
        qpos_idx = self._qpos_indices[axis_name]
        return round(float(data.qpos[qpos_idx]),3)

    def get_swing_angles(self, data: mujoco.MjData) -> Tuple[float, float]:
        """Get swing angles from ball joint (rope swing).

        Returns:
            Tuple of (swing_x, swing_y) in radians
            swing_x: Swing angle in X direction (roll)
            swing_y: Swing angle in Y direction (pitch)
        """
        if self._ball_joint_id is None:
            return 0.0, 0.0

        # Get quaternion from qpos [w, x, y, z]
        qpos_addr = self._ball_qpos_addr
        quat = data.qpos[qpos_addr:qpos_addr+4]

        # Convert quaternion to euler angles (roll, pitch, yaw)
        # Using ZYX convention
        w, x, y, z = quat

        # Roll (rotation around X axis)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (rotation around Y axis)
        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)  # Clamp for numerical stability
        pitch = np.arcsin(sinp)

        return round(float(roll), 4), round(float(pitch), 4)

    def get_swing_angular_velocity(self, data: mujoco.MjData) -> Tuple[float, float]:
        """Get angular velocity from ball joint.

        Returns:
            Tuple of (wx, wy, wz) in rad/s
        """
        if self._ball_joint_id is None:
            return 0.0, 0.0

        # Get angular velocity from qvel [wx, wy, wz]
        qvel_addr = self._ball_qvel_addr
        angvel = data.qvel[qvel_addr:qvel_addr+3]

        return round(float(angvel[0]), 4), round(float(angvel[1]), 4)

    def get_swing_data(self, data: mujoco.MjData) -> Dict:
        """Get all swing-related data.

        Returns:
            Dictionary with swing angles and angular velocities
        """
        swing_x, swing_y = self.get_swing_angles(data)
        wx, wy = self.get_swing_angular_velocity(data)

        return {
            'swing_x': swing_x,      # Swing angle in X direction (rad)
            'swing_y': swing_y,      # Swing angle in Y direction (rad)
            'ang_vel_x': wx,         # Angular velocity X (rad/s)
            'ang_vel_y': wy,         # Angular velocity Y (rad/s)
        }
