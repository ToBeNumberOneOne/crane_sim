import mujoco

class MujocoModel:
    def __init__(self, xml_path: str, timestep: float):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)

    def step(self):
        mujoco.mj_step(self.model, self.data)
