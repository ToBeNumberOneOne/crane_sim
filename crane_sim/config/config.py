"""Configuration management for crane simulation.

This module loads and parses the crane.yaml configuration file,
providing strongly-typed configuration objects.
"""

import yaml
from dataclasses import dataclass
from typing import Tuple


@dataclass
class AxisConfig:
    """Configuration for a single crane axis."""
    actuator: str  # MuJoCo actuator name
    vmax: float  # Maximum velocity (m/s)
    amax: float  # Maximum acceleration (m/s²)
    offset: float  # Position offset for coordinate transformation
    soft_limit: Tuple[float, float]  # Soft limit range (min, max)


@dataclass
class SimConfig:
    """Simulation configuration."""
    timestep: float  # Simulation timestep (seconds)
    realtime: bool  # Whether to run in real-time


class CraneConfig:
    """Main configuration loader for crane system."""

    def __init__(self, path: str):
        """Load configuration from YAML file.

        Args:
            path: Path to crane.yaml configuration file
        """
        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        # Load simulation config
        self.sim = SimConfig(**cfg["sim"])

        # Load axis configurations
        self.axes = {
            name: AxisConfig(**data)
            for name, data in cfg["axes"].items()
        }

        # Load telemetry config (simplified)
        telemetry_cfg = cfg.get("telemetry", {})
        self.publish_rate_hz = telemetry_cfg.get("publish_rate_hz", 10.0)

    @property
    def publish_interval(self) -> float:
        """Get publishing interval in seconds."""
        return 1.0 / self.publish_rate_hz
