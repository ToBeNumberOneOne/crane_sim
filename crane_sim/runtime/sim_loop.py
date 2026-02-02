import time

class SimulationLoop:
    def __init__(self, model, axes, state, ctrl):
        self.model = model
        self.axes = axes
        self.state = state
        self.ctrl = ctrl  

    def run(self, viewer):
        dt = self.model.model.opt.timestep
        next_time = time.perf_counter()

        while viewer.is_running():
            velocities = [getattr(self.ctrl, "vx", 0.0),
                          getattr(self.ctrl, "vy", 0.0),
                          getattr(self.ctrl, "vz", 0.0)] 
            for i, name in enumerate(["x", "y", "z"]):
                axis = self.axes[name]
                pos = self.model.data.qpos[i]
                axis.set_velocity(velocities[i])
                self.model.data.ctrl[axis.id] = axis.update(dt, pos)

            self.model.step()
            viewer.sync()
            self.state.sample(self.model.data, self.axes)

            # 强制对齐真实时间
            next_time += dt
            sleep = next_time - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)