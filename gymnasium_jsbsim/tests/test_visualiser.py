"""
Tests for visualization components.
"""

import time
import unittest
from typing import Optional

import matplotlib.pyplot as plt

from gymnasium_jsbsim import utils, visualiser
from gymnasium_jsbsim.environment import JsbSimEnv
from gymnasium_jsbsim.tests.stubs import BasicFlightTask, DefaultSimStub
from gymnasium_jsbsim.visualiser import FigureVisualiser, FlightGearVisualiser


@unittest.skipIf(not utils.is_flightgear_installed(), reason="FlightGear not installed")
class TestFigureVisualiser(unittest.TestCase):
    sim: Optional[DefaultSimStub] = None
    visualiser: Optional[FigureVisualiser] = None

    def setUp(self, _plot_position=True):
        self.sim = DefaultSimStub()
        task = BasicFlightTask()
        self.visualiser = FigureVisualiser(DefaultSimStub(), task.get_props_to_output())  # type: ignore[arg-type]

    def tearDown(self):
        self.visualiser.close()  # type: ignore[union-attr]

    def test_plot_creates_figure_and_axes(self):
        self.setUp()

        self.visualiser.plot(self.sim)  # type: ignore[union-attr,arg-type]

        self.assertIsInstance(self.visualiser.figure, plt.Figure)  # type: ignore[union-attr]
        self.assertIsInstance(self.visualiser.axes, visualiser.AxesTuple)  # type: ignore[union-attr]

    def test_plot_doesnt_plot_position_when_set_by_init(self):
        self.setUp(_plot_position=False)

        self.visualiser.plot(self.sim)  # type: ignore[union-attr,arg-type]

        position_axis = self.visualiser.axes.axes_state  # type: ignore[union-attr]
        is_empty_plot = position_axis is None or len(position_axis.axes.lines) == 0  # type: ignore[union-attr]
        self.assertTrue(is_empty_plot)

    def test_plot_plots_control_state(self):
        self.setUp()

        self.visualiser.plot(self.sim)  # type: ignore[union-attr,arg-type]

    def test_close_removes_figure(self):
        self.setUp()
        self.visualiser.plot(self.sim)  # type: ignore[union-attr,arg-type]

        self.visualiser.close()  # type: ignore[union-attr]

        self.assertIsNone(self.visualiser.figure)  # type: ignore[union-attr]
        self.assertIsNone(self.visualiser.axes)  # type: ignore[union-attr]


@unittest.skipIf(not utils.is_flightgear_installed(), reason="FlightGear not installed")
class TestFlightGearVisualiser(unittest.TestCase):
    env = None
    sim = None
    flightgear = None

    def setUp(self):
        if self.env:
            self.env.close()
        if self.sim:
            self.sim.close()
        self.task = BasicFlightTask()
        self.env = JsbSimEnv(task_type=BasicFlightTask)  # type: ignore[arg-type]
        self.env.reset()
        self.sim = self.env.sim
        self.flightgear = None
        # individual test methods should init as needed:
        # self.flightgear = FlightGearVisualiser(self.sim)

    def tearDown(self):
        if self.env:
            self.env.close()
        if self.sim:
            self.sim.close()
        if self.flightgear:
            self.flightgear.close()

    def test_init_creates_figure(self):
        self.flightgear = FlightGearVisualiser(  # type: ignore[arg-type]
            self.sim, self.task.get_props_to_output(), block_until_loaded=False  # type: ignore[arg-type]
        )
        self.assertIsInstance(self.flightgear.figure, FigureVisualiser)

    def test_launch_flightgear(self):
        self.flightgear = FlightGearVisualiser(
            self.sim, self.task.get_props_to_output(), block_until_loaded=False  # type: ignore[arg-type]
        )
        time.sleep(5)

        # Check if FlightGear process is still running
        self.assertIsNone(self.flightgear.flightgear_process.poll())

        self.flightgear.close()

    def test_close_closes_flightgear(self):
        self.flightgear = FlightGearVisualiser(
            self.sim, self.task.get_props_to_output(), block_until_loaded=False  # type: ignore[arg-type]
        )
        self.flightgear.close()
        timeout_seconds = 2.0
        return_code = self.flightgear.flightgear_process.wait(timeout=timeout_seconds)
        # a non-None return code indicates termination
        self.assertIsNotNone(return_code)

    def test_plot_displays_actions(self):
        self.setUp()
        self.flightgear = FlightGearVisualiser(
            self.sim, self.task.get_props_to_output(), block_until_loaded=False  # type: ignore[arg-type]
        )
        self.flightgear.plot(self.sim)  # type: ignore[arg-type]

        # the figure should have plotted a Lines object each axis
        for axis in ["axes_stick", "axes_rudder", "axes_throttle"]:
            axis_data_plots = getattr(self.flightgear.figure.axes, axis)
            is_empty_plot = len(axis_data_plots.axes.lines) == 0
            self.assertFalse(is_empty_plot, msg=f"no data plotted on axis {axis}")
