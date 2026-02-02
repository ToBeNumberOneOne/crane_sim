class CLI:
    def __init__(self, ctrl, state=None):
        self.ctrl = ctrl
        self.state = state
        width_list = [10, 12, 20]
        self.help_info = "\n --------- HELP INFO --------- "
        self.help_info += "\n {:<{}}".format("cmd", width_list[0])
        self.help_info += "{:^{}}".format("simple cmd", width_list[1])
        self.help_info += "{:<{}}".format("describ", width_list[2])

        self.cmd_list = []
        self.cmd_list.append(["h", "help", "帮助信息", self._help])
        self.cmd_list.append(["v", "velocity", "设置速度 v vx vy vz", self._velocity])
        self.cmd_list.append(["vx", "velocity_x", "设置x轴速度 vx value", lambda args: self._set_velocity(args, "x")])
        self.cmd_list.append(["vy", "velocity_y", "设置y轴速度 vy value", lambda args: self._set_velocity(args, "y")])
        self.cmd_list.append(["vz", "velocity_z", "设置z轴速度 vz value", lambda args: self._set_velocity(args, "z")])
        self.cmd_list.append(["1","get_state","获取当前状态",self._get_state])
        self.cmd_list.append(["s", "stop", "停止", self._stop])
        

        for it in self.cmd_list:
            self.help_info += "\n {:<{}}".format(it[1], width_list[0])
            self.help_info += "{:^{}}".format(it[0], width_list[1])
            self.help_info += "{:<{}}".format(it[2], width_list[2])

    def _help(self, args=None):
        print(self.help_info)

    def run(self):
        self._help()
        while True:
            cmd_line = input("please input cmd: ").strip()
            if not cmd_line:
                continue
            parts = cmd_line.split()
            cmd = parts[0]
            args = parts[1:]
            found = False
            for it in self.cmd_list:
                if it[0] == cmd or it[1] == cmd:
                    it[3](args)
                    found = True
                    break
            if not found:
                print("未知命令，输入 h 或 help 查看帮助")

    def _velocity(self, args):
        if len(args) != 3:
            print("用法: v vx vy vz")
            return
        try:
            vx, vy, vz = map(float, args)
            self.ctrl.vx = vx
            self.ctrl.vy = vy
            self.ctrl.vz = vz
            print(f"设置速度: vx={vx}, vy={vy}, vz={vz}")
        except Exception:
            print("参数错误，用法: v vx vy vz")

    def _set_velocity(self, args, axis):
        axis_map = {'x': 'vx', 'y': 'vy', 'z': 'vz'}
        if axis not in axis_map:
            print("未知轴命令")
            return
        if len(args) != 1:
            print(f"用法: {axis_map[axis]} value")
            return
        try:
            value = float(args[0])
            setattr(self.ctrl, axis_map[axis], value)
            print(f"设置{axis_map[axis]}轴速度: {axis_map[axis]}={value}")
        except Exception:
            print(f"参数错误，用法: {axis_map[axis]} value")


    def _get_state(self, args=None):
        if self.state is None:
            print("未找到状态对象 (state)，请确保已正确传入")
            return
        pos = getattr(self.state, 'pos', {})
        vel = getattr(self.state, 'vel', {})
        cmd_vel = getattr(self.state, 'cmd_vel', {})
        def fmt(val):
            try:
                return f"{float(val):.3f}"
            except:
                return str(val)
        print("\n  当前起重机状态")
        print("  {:<8} {:<8} {:<8} {:<8}".format("物理量", "x轴", "y轴", "z轴"))
        print("  {:<8} {:<8} {:<8} {:<8}".format("位置(m)", fmt(pos.get('x','N/A')), fmt(pos.get('y','N/A')), fmt(pos.get('z','N/A'))))
        print("  {:<8} {:<8} {:<8} {:<8}".format("速度(m/s)", fmt(vel.get('x','N/A')), fmt(vel.get('y','N/A')), fmt(vel.get('z','N/A'))))
        print("  {:<8} {:<8} {:<8} {:<8}".format("指令(m/s)", fmt(cmd_vel.get('x','N/A')), fmt(cmd_vel.get('y','N/A')), fmt(cmd_vel.get('z','N/A'))))
        print()


    def _stop(self, args=None):
            self.ctrl.vx = 0
            self.ctrl.vy = 0
            self.ctrl.vz = 0
            print("停止运动")