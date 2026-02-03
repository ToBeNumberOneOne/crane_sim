"""Simulation loop for crane control.

This module implements the main simulation loop with clear separation
of responsibilities: control, physics, visualization, monitoring, and timing.
"""

import time


class SimulationLoop:
    """Main simulation loop with separated concerns.

    Coordinates control commands, physics simulation, visualization,
    state monitoring, and real-time synchronization.
    """

    def __init__(self, model, axes, state, mapper, cli, xbox=None):
        """Initialize the simulation loop.

        Args:
            model: MujocoModel instance
            axes: Dictionary of AxisController instances
            state: CraneState instance for monitoring
            mapper: JointMapper for joint position/velocity access
            cli: CLI instance (for quit flag checking)
            xbox: XboxController instance (optional)
        """
        self.model = model
        self.axes = axes
        self.state = state
        self.mapper = mapper
        self.cli = cli
        self.xbox = xbox

    def run(self, viewer):
        """Main simulation loop.

        Args:
            viewer: MuJoCo viewer instance
        """
        dt = self.model.model.opt.timestep
        next_time = time.perf_counter()

        print("Simulation loop started. Press Ctrl+C to exit.")

        while viewer.is_running() and not self.cli.is_quit_requested():
            # Separate concerns into distinct methods
            self._control_step(dt)
            self._physics_step()
            self._visualization_step(viewer)
            self._monitor_step()
            self._timing_step(dt, next_time)
            next_time += dt

        print("Simulation loop ended.")

    def _control_step(self, dt):
        """Execute control logic for all axes.

        Each AxisController is directly updated and returns its control output.

        Args:
            dt: Time step in seconds
        """
        # Update Xbox controller in main thread (pygame requirement)
        if self.xbox:
            self.xbox.update()

        for axis in self.axes.values():
            # AxisController.update() is thread-safe and handles everything internally
            ctrl_value = axis.update(dt, self.mapper, self.model.data)
            self.model.data.ctrl[axis.id] = ctrl_value
            # z轴重力补偿
            if axis.axis_name == 'z':
                self.model.data.ctrl[axis.id] += 9.81 * 1.0 / 200

    def _physics_step(self):
        """Step the physics simulation forward."""
        self.model.step()

    def _visualization_step(self, viewer):
        """Update the visualization.

        Args:
            viewer: MuJoCo viewer instance
        """
        viewer.sync()

    def _monitor_step(self):
        """Collect and store state data for monitoring."""
        self.state.sample(self.mapper, self.model.data, self.axes)

    def _timing_step(self, dt, next_time):
        """Maintain real-time synchronization.

        Args:
            dt: Time step in seconds
            next_time: Target time for next step
        """
        sleep = next_time - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)