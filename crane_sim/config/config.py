import yaml
from dataclasses import dataclass

@dataclass
class AxisConfig:
    actuator: str
    vmax: float
    amax: float
    offset: float
    soft_limit: tuple

@dataclass
class ControlContext:
    # 模式
    mode: str = "manual" 

    # 速度给定
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 5 * 9.81 / 40  # 默认补偿重力

    # 事件
    stop: bool = False
    quit: bool = False

@dataclass
class SimConfig:
    timestep: float
    realtime: bool

class CraneConfig:
    def __init__(self, path: str):
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)

        self.sim = SimConfig(**cfg["sim"])
        self.axes = {
            name: AxisConfig(**data)
            for name, data in cfg["axes"].items()
        }
