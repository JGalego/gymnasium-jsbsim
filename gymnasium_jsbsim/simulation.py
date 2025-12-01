"""
Wrapper for JSBSim flight dynamics simulator.
"""

import os
import time
from typing import Dict, Union

import jsbsim

import gymnasium_jsbsim.properties as prp
from gymnasium_jsbsim import constants
from gymnasium_jsbsim.aircraft import Aircraft, cessna172P


class Simulation:
    """
    Wrapper class for JSBSim flight dynamics simulator.
    """

    # pylint: disable=too-many-arguments,too-many-positional-arguments
    def __init__(
        self,
        sim_frequency_hz: float = 60.0,
        aircraft: Aircraft = cessna172P,
        init_conditions: Dict[prp.Property, float] = None,
        allow_flightgear_output: bool = True,
        root_dir: Union[str, None] = None,
    ):
        """
        Creates and initialises a JSBSim simulation instance.

        Args:
            sim_frequency_hz: the JSBSim integration frequency in Hz.
            aircraft: Aircraft instance to be loaded.
            init_conditions: dict mapping properties to their initial values.
                Defaults to None, causing a default set of initial props to be used.
            allow_flightgear_output: bool, loads a config file instructing
                JSBSim to connect to an output socket if True.
            root_dir: JSBSim root directory path
        """
        if init_conditions is None:
            init_conditions = {}
        self.jsbsim = jsbsim.FGFDMExec(root_dir)
        self.jsbsim.set_debug_level(0)
        if allow_flightgear_output:
            flightgear_output_config = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), constants.OUTPUT_FILE
            )
            self.jsbsim.set_output_directive(flightgear_output_config)
        self.sim_dt = 1.0 / sim_frequency_hz
        self.aircraft = aircraft
        self.initialise(self.sim_dt, self.aircraft.jsbsim_id, init_conditions)
        self.jsbsim.disable_output()
        self.wall_clock_dt = None

    def __getitem__(self, prop: Union[prp.BoundedProperty, prp.Property]) -> float:
        """
        Gets simulation property value.

        Properties are identified by strings. A list can be found in the JSBSim
        reference manual, launching JSBSim with '--catalog' command line arg or
        calling FGFDMExec.get_property_catalog().

        :param prop: BoundedProperty, the property to be retrieved
        :return: float
        """
        return self.jsbsim[prop.name]

    def __setitem__(
        self, prop: Union[prp.BoundedProperty, prp.Property], value
    ) -> None:
        """
        Sets simulation property to specified value.

        Properties are identified by strings. A list can be found in the JSBSim
        reference manual, launching JSBSim with '--catalog' command line arg or
        calling FGFDMExec.get_property_catalog().

        Warning: JSBSim will create new properties if the specified one exists.
        If the property you are setting is read-only in JSBSim the operation
        will silently fail.

        :param prop: BoundedProperty, the property to be retrieved
        :param value: object?, the value to be set
        """
        self.jsbsim[prop.name] = value

    def load_model(self, model_name: str) -> None:
        """
        Loads the specified aircraft config into the simulation.

        The root JSBSim directory aircraft folder is searched for the aircraft
        XML config file.

        :param model_name: string, the aircraft name
        """
        load_success = self.jsbsim.load_model(model_name)

        if not load_success:
            raise RuntimeError(
                "JSBSim could not find specified model_name: " + model_name
            )

    def get_aircraft(self) -> Aircraft:
        """
        Gets the Aircraft this sim was initialised with.
        """
        return self.aircraft

    def get_loaded_model_name(self) -> str:
        """
        Gets the name of the aircraft model currently loaded in JSBSim.

        Returns:
            string, the name of the aircraft model if one is loaded, or
            None if no model is loaded.
        """
        name: str = self.jsbsim.get_model_name()
        if name:
            return name
        # Name is empty string, no model is loaded
        return None

    def get_sim_time(self) -> float:
        """
        Gets the simulation time from JSBSim, a float.

        Returns:
            Current simulation time in seconds
        """
        return self.jsbsim["simulation/sim-time-sec"]

    def initialise(
        self,
        dt: float,
        model_name: str,
        init_conditions: Dict["prp.Property", float] = None,
    ) -> None:
        """
        Initialises JSBSim with the specified aircraft and initial conditions.

        JSBSim creates an InitialConditions object internally when given an
        XML config file. This method either loads a basic set of ICs, or
        can be passed a dictionary with ICs. In the latter case a minimal IC
        XML file is loaded, and then the dictionary values are fed in.

        Args:
            dt: float, the JSBSim integration timestep in seconds
            model_name: string, name of aircraft to be loaded
            init_conditions: dict mapping properties to their initial values
        """
        if init_conditions:
            # if we are specifying conditions, load a minimal file
            ic_file = "minimal_ic.xml"
        else:
            ic_file = "basic_ic.xml"

        ic_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ic_file)
        self.jsbsim.load_ic(ic_path, useAircraftPath=False)
        self.load_model(model_name)
        self.jsbsim.set_dt(dt)
        # extract set of legal property names for this aircraft
        # Note: JSBSim property extraction is handled by the underlying library

        # now that IC object is created in JSBSim, specify own conditions
        self.set_custom_initial_conditions(init_conditions)

        success = self.jsbsim.run_ic()
        if not success:
            raise RuntimeError("JSBSim failed to init simulation conditions.")

    def set_custom_initial_conditions(
        self, init_conditions: Dict["prp.Property", float] = None
    ) -> None:
        """
        Set custom initial conditions in the simulation.

        Args:
            init_conditions: Dictionary mapping properties to their initial values
        """
        if init_conditions is not None:
            for prop, value in init_conditions.items():
                self[prop] = value

    def reinitialise(self, init_conditions: Dict["prp.Property", float] = None) -> None:
        """
        Reinitialises the JSBSim simulation to the initial conditions.

        The same aircraft and other settings are kept loaded in JSBSim. If a
        dict of ICs is provided, JSBSim is initialised using these, else the
        last specified ICs are used.

        :param init_conditions: dict mapping properties to their initial values
        """
        self.set_custom_initial_conditions(init_conditions=init_conditions)
        no_output_reset_mode = 0
        self.jsbsim.reset_to_initial_conditions(no_output_reset_mode)

    def run(self) -> bool:
        """
        Advances the simulation by one timestep.

        JSBSim monitors the simulation and detects whether it thinks it should
        end, e.g. because a simulation time was specified. False is returned
        if JSBSim termination criteria are met.

        :return: bool, False if sim has met JSBSim termination criteria else True.
        """
        result = self.jsbsim.run()
        if self.wall_clock_dt is not None:
            time.sleep(self.wall_clock_dt)
        return result

    def enable_flightgear_output(self):
        """
        Enables output from the FlightGear simulator.
        """
        self.jsbsim.enable_output()

    def disable_flightgear_output(self):
        """
        Disables output from the FlightGear simulator.
        """
        self.jsbsim.disable_output()

    def close(self):
        """
        Closes the simulation and any plots.
        """
        if self.jsbsim:
            self.jsbsim = None

    def set_simulation_time_factor(self, time_factor):
        """
        Sets the simulation speed relative to realtime.

        The simulation runs at realtime for time_factor = 1. It runs at double
        speed for time_factor=2, and half speed for 0.5.

        :param time_factor: int or float, nonzero, sim speed relative to realtime
            if None, the simulation is run at maximum computational speed
        """
        if time_factor is None:
            self.wall_clock_dt = None
        elif time_factor <= 0:
            raise ValueError("time factor must be positive and non-zero")
        else:
            self.wall_clock_dt = self.sim_dt / time_factor

    def start_engines(self):
        """
        Starts all aircraft engines.
        """
        self[prp.all_engine_running] = -1

    def set_throttle_mixture_controls(self, throttle_cmd: float, mixture_cmd: float):
        """
        Sets the throttle and mixture commands for the aircraft.

        If an aircraft is multi-engine and has multiple throttle_cmd and mixture_cmd
        controls, sets all of them. Currently only supports up to two throttles/mixtures.
        """
        self[prp.throttle_cmd] = throttle_cmd
        self[prp.mixture_cmd] = mixture_cmd

        try:
            self[prp.throttle_1_cmd] = throttle_cmd
            self[prp.mixture_1_cmd] = mixture_cmd
        except KeyError:
            pass  # must be single-control aircraft

    def raise_landing_gear(self):
        """
        Raises the aircraft landing gear.
        """
        self[prp.gear] = 0.0
        self[prp.gear_all_cmd] = 0.0
