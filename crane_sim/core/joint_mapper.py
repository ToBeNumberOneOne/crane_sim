"""Joint to qpos/qvel index mapper for MuJoCo model.

This module provides a unified interface to access joint positions and velocities
from MuJoCo's qpos/qvel arrays, handling complex joint types (ball joints, etc.)
and applying configuration offsets.
"""

import mujoco
from typing import Dict
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

        # Log the mappings
        print("\n=== Joint Mapper Initialized ===")
        for axis_name in joint_configs.keys():
            joint_id = self._joint_ids[axis_name]
            qpos_idx = self._qpos_indices[axis_name]
            qvel_idx = self._qvel_indices[axis_name]
            joint_name = model.joint(joint_id).name
            print(f"  {axis_name} axis: joint '{joint_name}' (id={joint_id}) -> qpos[{qpos_idx}], qvel[{qvel_idx}]")
        print("================================\n")

    def get_position(self, data: mujoco.MjData, axis_name: str) -> float:
        """Get the position of a specific axis, with offset applied."""
        qpos_idx = self._qpos_indices[axis_name]
        offset = self.joint_configs[axis_name].offset
        return float(data.qpos[qpos_idx] + offset)

    def get_velocity(self, data: mujoco.MjData, axis_name: str) -> float:
        """Get the velocity of a specific axis."""
        qvel_idx = self._qvel_indices[axis_name]
        return float(data.qvel[qvel_idx])

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
        return float(data.qpos[qpos_idx])
