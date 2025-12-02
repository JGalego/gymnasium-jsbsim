"""Module for visualisation of JSBSim simulations using FlightGear and Matplotlib."""

import subprocess
import time
from typing import NamedTuple, Optional, Tuple

import matplotlib.pyplot as plt

import gymnasium_jsbsim.properties as prp
from gymnasium_jsbsim import constants
from gymnasium_jsbsim.aircraft import Aircraft
from gymnasium_jsbsim.simulation import Simulation


class AxesTuple(NamedTuple):
    """Holds references to figure subplots (axes)"""

    axes_state: plt.Axes
    axes_stick: plt.Axes
    axes_throttle: plt.Axes
    axes_rudder: plt.Axes


class FigureVisualiser:
    """Class for manging a matplotlib Figure displaying agent state and actions"""

    def __init__(self, _: Simulation, print_props: Tuple[prp.Property]):
        """
        Constructor.

        Sets here is ft_per_deg_lon, which depends dynamically on aircraft's
        longitude (because of the conversion between geographic and Euclidean
        coordinate systems). We retrieve longitude from the simulation and
        assume it is constant thereafter.

        :param _: (unused) Simulation that will be plotted
        :param print_props: Propertys which will have their values printed to Figure.
            Must be retrievable from the plotted Simulation.
        """
        self.print_props = print_props
        self.figure: Optional[plt.Figure] = None
        self.axes: Optional[AxesTuple] = None
        self.value_texts: Optional[Tuple[plt.Text, ...]] = None

    def plot(self, sim: Simulation) -> None:
        """
        Creates or updates a 3D plot of the episode.

        :param sim: Simulation that will be plotted
        """
        if not self.figure:
            self.figure, self.axes = self._plot_configure()

        # Delete old control surface data points
        if self.axes:
            for subplot in self.axes[1:]:  # type: ignore[index]
                # Pop and translate all data points
                while subplot.lines:
                    subplot.lines[0].remove()  # type: ignore[func-returns-value]

        self._print_state(sim)
        if self.axes:
            self._plot_control_states(sim, self.axes)
            self._plot_control_commands(sim, self.axes)
        plt.pause(
            constants.PLOT_PAUSE_SECONDS
        )  # Voodoo pause needed for figure to update

    def close(self):
        """Clean up and close the figure."""
        if self.figure:
            plt.close(self.figure)
            self.figure = None  # type: ignore[assignment]
            self.axes = None  # type: ignore[assignment]

    def _plot_configure(self):
        """
        Creates a figure with subplots for states and actions.

        :return: (figure, axes) where:
            figure: a matplotlib Figure with subplots for state and controls
            axes: an AxesTuple object with references to all figure subplot axes
        """
        plt.ion()  # Interactive mode allows dynamic updating of plot
        figure = plt.figure(figsize=(6, 11))
        spec = plt.GridSpec(
            nrows=3, ncols=2, width_ratios=[5, 1], height_ratios=[6, 5, 1], wspace=0.3
        )

        # Create subplots
        axes_state = figure.add_subplot(spec[0, 0:])
        axes_stick = figure.add_subplot(spec[1, 0])
        axes_throttle = figure.add_subplot(spec[1, 1])
        axes_rudder = figure.add_subplot(spec[2, 0])

        # Configure state subplot
        axes_state.axis("off")
        self._prepare_state_printing(axes_state)

        # Shared configuration
        minor_locator = plt.MultipleLocator(0.2)

        # Configure stick subplot (2D: aileron and elevator)
        axes_stick.set(
            xlabel="ailerons [-]",
            ylabel="elevator [-]",
            xlim=(-1, 1),
            ylim=(-1, 1),
            xticks=[-1, 1],
            yticks=[-1, 1],
        )
        axes_stick.xaxis.set_label_coords(0.5, 1.08)
        axes_stick.yaxis.set_label_coords(-0.05, 0.5)
        axes_stick.spines["left"].set_position("zero")
        axes_stick.spines["bottom"].set_position("zero")
        axes_stick.xaxis.set_ticks_position("bottom")
        axes_stick.yaxis.set_ticks_position("left")
        axes_stick.tick_params(which="both", direction="inout")
        axes_stick.xaxis.set_minor_locator(minor_locator)
        axes_stick.yaxis.set_minor_locator(minor_locator)
        for spine in ["right", "top"]:
            axes_stick.spines[spine].set_visible(False)

        # Configure throttle subplot (1D vertical)
        axes_throttle.set(
            ylabel="throttle [-]", ylim=(0, 1), xlim=(0, 1), yticks=[0, 0.5, 1]
        )
        axes_throttle.spines["left"].set_position("zero")
        axes_throttle.yaxis.set_label_coords(0.5, 0.5)
        axes_throttle.yaxis.set_minor_locator(minor_locator)
        axes_throttle.tick_params(axis="y", which="both", direction="inout")
        axes_throttle.xaxis.set_visible(False)
        for spine in ["right", "bottom", "top"]:
            axes_throttle.spines[spine].set_visible(False)

        # Configure rudder subplot (1D horizontal)
        axes_rudder.set(
            xlabel="rudder [-]", xlim=(-1, 1), ylim=(0, 1), xticks=[-1, 0, 1]
        )
        axes_rudder.xaxis.set_label_coords(0.5, -0.5)
        axes_rudder.spines["bottom"].set_position("zero")
        axes_rudder.xaxis.set_minor_locator(minor_locator)
        axes_rudder.tick_params(axis="x", which="both", direction="inout")
        axes_rudder.get_yaxis().set_visible(False)
        for spine in ["left", "right", "top"]:
            axes_rudder.spines[spine].set_visible(False)

        all_axes = AxesTuple(axes_state, axes_stick, axes_throttle, axes_rudder)

        # Create figure-wide legend
        cmd_line = plt.Line2D(
            [], [], color="b", marker="o", ms=10, linestyle="", fillstyle="none"
        )
        pos_line = plt.Line2D([], [], color="r", marker="+", ms=10, linestyle="")
        figure.legend(
            [cmd_line, pos_line],
            ["Commanded Position, normalised", "Current Position, normalised"],
            loc="lower center",
        )

        plt.show()
        plt.pause(
            constants.PLOT_PAUSE_SECONDS
        )  # Voodoo pause needed for figure to appear

        return figure, all_axes

    def _prepare_state_printing(self, ax: plt.Axes):
        ys = [
            constants.TEXT_Y_POSN_INITIAL + i * constants.TEXT_Y_INCREMENT
            for i in range(len(self.print_props))
        ]

        for prop, y in zip(self.print_props, ys):
            label = str(prop.name)
            ax.text(
                constants.TEXT_X_POSN_LABEL,
                y,
                label,
                transform=ax.transAxes,
                **constants.LABEL_TEXT_KWARGS,  # type: ignore[arg-type]
            )

        # Print and store empty Text objects which we will rewrite each plot call
        value_texts = []
        dummy_msg = ""
        for y in ys:
            text = ax.text(
                constants.TEXT_X_POSN_VALUE,
                y,
                dummy_msg,
                transform=ax.transAxes,
                **constants.VALUE_TEXT_KWARGS,  # type: ignore[arg-type]
            )
            value_texts.append(text)
        self.value_texts = tuple(value_texts)  # type: ignore[assignment]

    def _print_state(self, sim: Simulation):
        # Update each Text object with latest value
        if self.value_texts:
            for prop, text in zip(self.print_props, self.value_texts):
                text.set_text(f"{sim[prop]:.4g}")

    def _plot_control_states(self, sim: Simulation, all_axes: AxesTuple):
        control_surfaces = [prp.aileron_left, prp.elevator, prp.throttle, prp.rudder]
        ail, ele, thr, rud = [sim[control] for control in control_surfaces]
        # Plot aircraft control surface positions
        all_axes.axes_stick.plot(
            [ail], [ele], "r+", mfc="none", markersize=10, clip_on=False
        )
        all_axes.axes_throttle.plot(
            [0], [thr], "r+", mfc="none", markersize=10, clip_on=False
        )
        all_axes.axes_rudder.plot(
            [rud], [0], "r+", mfc="none", markersize=10, clip_on=False
        )

    def _plot_control_commands(self, sim: Simulation, all_axes: AxesTuple):
        """
        Plots agent-commanded actions on the environment figure.

        :param sim: Simulation to plot control commands from
        :param all_axes: AxesTuple, collection of axes of subplots to plot on
        """
        ail_cmd = sim[prp.aileron_cmd]
        ele_cmd = sim[prp.elevator_cmd]
        thr_cmd = sim[prp.throttle_cmd]
        rud_cmd = sim[prp.rudder_cmd]

        all_axes.axes_stick.plot(
            [ail_cmd], [ele_cmd], "bo", mfc="none", markersize=10, clip_on=False
        )
        all_axes.axes_throttle.plot(
            [0], [thr_cmd], "bo", mfc="none", markersize=10, clip_on=False
        )
        all_axes.axes_rudder.plot(
            [rud_cmd], [0], "bo", mfc="none", markersize=10, clip_on=False
        )


class FlightGearVisualiser:
    """
    Class for visualising aircraft using the FlightGear simulator.

    This visualiser launches FlightGear and (by default) waits for it to
    launch. A Figure is also displayed (by creating its own FigureVisualiser)
    which is used to display the agent's actions.
    """

    def __init__(
        self, sim: Simulation, print_props: Tuple[prp.Property], block_until_loaded=True
    ):
        """
        Launches FlightGear in subprocess and starts figure for plotting actions.

        :param sim: Simulation that will be visualised
        :param aircraft: Aircraft to be loaded in FlightGear for visualisation
        :param print_props: collection of Propertys to be printed to Figure
        :param block_until_loaded: visualiser will block until it detects that
            FlightGear has loaded if True.
        """
        self.configure_simulation_output(sim)
        self.print_props = print_props
        # Note: subprocess is managed manually (not with context manager)
        # Because it needs to stay alive for the visualiser's lifetime
        # And is explicitly closed in close() method
        self.flightgear_process = self._launch_flightgear(sim.get_aircraft())
        self.figure = FigureVisualiser(sim, print_props)
        if block_until_loaded:
            time.sleep(20)
            # self._block_until_flightgear_loaded()

    def plot(self, sim: Simulation) -> None:
        """
        Updates a 3D plot of agent actions.
        """
        self.figure.plot(sim)

    @staticmethod
    def _launch_flightgear(aircraft: Aircraft):
        """Launch FlightGear subprocess for visualization."""
        cmd_line_args = FlightGearVisualiser._create_cmd_line_args(
            aircraft.flightgear_id
        )
        # Subprocess is not used with context manager because it needs to persist
        # And is managed manually via close() method

        flightgear_process = subprocess.Popen(
            cmd_line_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        return flightgear_process

    def configure_simulation_output(self, sim: Simulation):
        """Configure simulation for FlightGear output."""
        sim.enable_flightgear_output()
        sim.set_simulation_time_factor(constants.FG_TIME_FACTOR)

    @staticmethod
    def _create_cmd_line_args(aircraft_id: str):
        # FlightGear doesn't have a 172X model, use the P instead
        if aircraft_id == "c172x":
            aircraft_id = "c172p"

        flightgear_cmd = "fgfs"
        aircraft_arg = f"--aircraft={aircraft_id}"
        flight_model_arg = (
            "--native-fdm=" + f"{constants.FG_TYPE},"
            f"{constants.FG_DIRECTION},"
            f"{constants.FG_RATE},"
            f"{constants.FG_SERVER},"
            f"{constants.FG_PORT},"
            f"{constants.FG_PROTOCOL}"
        )
        flight_model_type_arg = "--fdm=" + "external"
        disable_ai_arg = "--disable-ai-traffic"
        disable_live_weather_arg = "--disable-real-weather-fetch"
        time_of_day_arg = "--timeofday=" + constants.FG_TIME
        return (
            flightgear_cmd,
            aircraft_arg,
            flight_model_arg,
            flight_model_type_arg,
            disable_ai_arg,
            disable_live_weather_arg,
            time_of_day_arg,
        )

    def _block_until_flightgear_loaded(self):
        """Wait until FlightGear has finished loading."""
        while True:
            msg_out = self.flightgear_process.stdout.readline().decode()
            if constants.FG_LOADED_MESSAGE in msg_out:
                break
            time.sleep(0.001)

    def close(self):
        """Close FlightGear and figure visualiser."""
        if self.flightgear_process:
            self.flightgear_process.kill()
            timeout_secs = 1
            self.flightgear_process.wait(timeout=timeout_secs)
