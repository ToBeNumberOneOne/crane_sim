import mujoco

class AxisController:
    def __init__(self, model, axis_cfg):
        self.id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, axis_cfg.actuator
        )
        self.cfg = axis_cfg
        self.cmd_v = 0.0
        self.cur_v = 0.0

    def set_velocity(self, v):
        self.cmd_v = max(
            -self.cfg.vmax, min(self.cfg.vmax, v)
        )

    def update(self, dt, pos):
        # 软限位
        if pos < self.cfg.soft_limit[0] and self.cmd_v < 0:
            self.cmd_v = 0
        if pos > self.cfg.soft_limit[1] and self.cmd_v > 0:
            self.cmd_v = 0

        # 简单加速度限制（后续可被 planner 接管）
        dv = self.cfg.amax * dt
        self.cur_v += max(-dv, min(dv, self.cmd_v - self.cur_v))

        return self.cur_v
