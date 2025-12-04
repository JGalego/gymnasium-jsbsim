"""
Gymnasium environment wrappers for JSBSim flight simulation.

This module provides `Env`-compliant interfaces to JSBSim for reinforcement
learning (RL) applications with optional FlightGear (FG) visualization support.
"""

from typing import Dict, Optional, Tuple, Type, Union

import gymnasium as gym
import numpy as np

from gymnasium_jsbsim import constants
from gymnasium_jsbsim.aircraft import Aircraft, cessna172P
from gymnasium_jsbsim.simulation import Simulation
from gymnasium_jsbsim.tasks import HeadingControlTask, Shaping
from gymnasium_jsbsim.visualiser import FigureVisualiser, FlightGearVisualiser

JSBSIM_DT_HZ: int = int(constants.JSBSIM_DT_HZ)  # JSBSim integration frequency


class JsbSimEnv(gym.Env):
    """
    Wrapper class for JSBSim based environments.

    `JsbSimEnv` is instantiated with a `Task` that implements a specific aircraft
    control task with its own specific observation/action space and variables
    and agent reward calculation.
    """

    metadata = {"render_modes": ["human", "flightgear"]}

    def __init__(
        self,
        task_type: Type[HeadingControlTask],
        aircraft: Aircraft = cessna172P,
        agent_interaction_freq: int = 5,
        shaping: Shaping = Shaping.STANDARD,
    ):
        """
        Initializes some internal state, but JsbSimEnv.reset() must be called
        first before interacting with environment.

        :param task_type: the `Task` subclass for the task agent is to perform
        :param aircraft: the JSBSim aircraft to be used
        :param agent_interaction_freq: int, how many times per second the agent
            should interact with environment.
        :param shaping: a `HeadingControlTask.Shaping` enum, what type of agent_reward
            shaping to use (see `HeadingControlTask` for options)
        """
        if agent_interaction_freq > constants.JSBSIM_DT_HZ:
            raise ValueError(
                "⛔ Agent interaction frequency must be less than "
                "or equal to JSBSim integration frequency of "
                f"{constants.JSBSIM_DT_HZ} Hz!"
            )
        self.sim: Optional[Simulation] = None
        self.sim_steps_per_agent_step: int = int(
            constants.JSBSIM_DT_HZ / agent_interaction_freq
        )
        self.aircraft = aircraft
        self.task = task_type(shaping, agent_interaction_freq, aircraft)

        # Set Space objects
        self.observation_space: gym.spaces.Box = self.task.get_state_space()  # type: ignore[assignment]
        self.action_space: gym.spaces.Box = self.task.get_action_space()  # type: ignore[assignment]

        # Set visualisation objects
        self.figure_visualiser: Optional[FigureVisualiser] = None
        self.flightgear_visualiser: Optional[FlightGearVisualiser] = None
        self.step_delay = None

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Run one timestep of the environment's dynamics.

        When end of episode is reached, you are responsible for calling `reset()`
        to reset this environment's state.

        Accepts an action and returns a tuple (observation, reward, terminated, truncated, info).

        :param action: the agent's action, with same length as action variables.
        :return:
            state: agent's observation of the current environment
            reward: amount of reward returned after previous action
            terminated: whether the episode reached a terminal state
            truncated: whether the episode was truncated (e.g., time limit)
            info: auxiliary information, e.g. full reward shaping data
        """
        if not action.shape == self.action_space.shape:
            raise ValueError("⛔ Mismatch found between action and action space size!")
        assert self.sim is not None, "Simulation not initialized - all reset() first!"

        state, reward, terminated, truncated, info = self.task.task_step(
            self.sim, action.tolist(), self.sim_steps_per_agent_step  # type: ignore[arg-type]
        )
        obs = np.array(state, dtype=np.float64)
        obs = np.array(state, dtype=np.float64)
        obs = np.clip(obs, self.observation_space.low, self.observation_space.high)
        return obs, reward, terminated, truncated, info

    # seed parameter required by Gymnasium API
    def reset(
        self, *, seed: Union[int, None] = None, options: Union[Dict, None] = None
    ):
        """
        Resets the environment and returns an initial observation and info.

        :return: tuple of (observation, info dict)
        """
        init_conditions = self.task.get_initial_conditions()
        if self.sim:
            self.sim.reinitialise(init_conditions)
        else:
            self.sim = self._init_new_sim(
                constants.JSBSIM_DT_HZ, self.aircraft, init_conditions
            )

        state = self.task.observe_first_state(self.sim)

        if self.flightgear_visualiser:
            self.flightgear_visualiser.configure_simulation_output(self.sim)

        obs = np.array(state, dtype=np.float64)
        obs = np.clip(obs, self.observation_space.low, self.observation_space.high)
        return obs, {}

    def _init_new_sim(self, dt, aircraft, initial_conditions):
        return Simulation(
            sim_frequency_hz=dt, aircraft=aircraft, init_conditions=initial_conditions
        )

    def render(self, mode="flightgear", flightgear_blocking=True):
        """Renders the environment.

        The set of supported modes varies per environment and some
        environments do not support rendering at all.

        By convention, if mode is:
        - `human`: render to the current display or terminal and
        return nothing. Usually for human consumption.
        - `rgb_array`: return a `numpy.ndarray` with shape `(x, y, 3)`,
        representing RGB values for an x-by-y pixel image, suitable
        for turning into a video.
        - `ansi`: return a string (str) or `StringIO.StringIO` containing a
        terminal-style text representation. The text can include newlines
        and ANSI escape sequences (e.g. for colors).

        Note: Make sure that your class's metadata 'render.modes' key includes
        the list of supported modes. It's recommended to call `super()` in
        implementations to use the functionality of this method.

        :param mode: str, the mode to render with
        :param flightgear_blocking: waits for FlightGear to load before
            returning if `True`, else returns immediately
        """
        if mode == "human":
            if not self.figure_visualiser:
                self.figure_visualiser = FigureVisualiser(
                    self.sim, self.task.get_props_to_output()  # type: ignore[arg-type]
                )
            self.figure_visualiser.plot(self.sim)  # type: ignore[arg-type]
        elif mode == "flightgear":
            if not self.flightgear_visualiser:
                self.flightgear_visualiser = FlightGearVisualiser(
                    self.sim, self.task.get_props_to_output(), flightgear_blocking  # type: ignore[arg-type]
                )
            self.flightgear_visualiser.plot(self.sim)  # type: ignore[arg-type]
        else:
            super().render()

    def close(self):
        """
        Cleans up this environment's objects.

        Environments automatically `close()` when garbage collected or when the
        program exits.
        """
        if self.sim:
            self.sim.close()
        if self.figure_visualiser:
            self.figure_visualiser.close()
        if self.flightgear_visualiser:
            self.flightgear_visualiser.close()

    def seed(self, seed=None):
        """
        Sets the `seed` for this environment's random number generator(s).

        Note:
            Some environments use multiple pseudorandom number generators.
            We want to capture all such seeds used in order to ensure that
            there aren't accidental correlations between multiple generators.
        """
        gym.logger.warn("Could not seed environment %s", self)


class NoFGJsbSimEnv(JsbSimEnv):
    """
    Wrapper class for JSBSim based environments with FlightGear disabled.

    This class exists to be used for training agents where visualisation is not
    required. Otherwise, restrictions in JSBSim output initialisation cause it
    to open a new socket for every single episode.
    """

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        task_type: Type[HeadingControlTask],
        aircraft: Aircraft = cessna172P,
        agent_interaction_freq: int = 5,
        shaping: Shaping = Shaping.STANDARD,
    ):
        super().__init__(task_type, aircraft, agent_interaction_freq, shaping)

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        if not action.shape == self.action_space.shape:
            raise ValueError("Mismatch between action and action space size!")

        assert (
            self.sim is not None
        ), "Simulation was not initialized - call reset() first!"

        state, reward, terminated, truncated, info = self.task.task_step(
            self.sim, action.tolist(), self.sim_steps_per_agent_step  # type: ignore[arg-type]
        )
        obs = np.array(state, dtype=np.float64)
        obs = np.clip(obs, self.observation_space.low, self.observation_space.high)
        return obs, reward, terminated, truncated, info

    # seed parameter required by Gymnasium API
    def reset(
        self, *, seed: Union[int, None] = None, options: Union[Dict, None] = None
    ):
        init_conditions = self.task.get_initial_conditions()
        if self.sim:
            self.sim.reinitialise(init_conditions)
        else:
            self.sim = self._init_new_sim(
                constants.JSBSIM_DT_HZ, self.aircraft, init_conditions
            )

        state = self.task.observe_first_state(self.sim)

        if self.flightgear_visualiser:
            self.flightgear_visualiser.configure_simulation_output(self.sim)

        obs = np.array(state, dtype=np.float64)
        obs = np.clip(obs, self.observation_space.low, self.observation_space.high)
        return obs, {}

    def _init_new_sim(self, dt: float, aircraft: Aircraft, initial_conditions: Dict):
        return Simulation(
            sim_frequency_hz=dt,
            aircraft=aircraft,
            init_conditions=initial_conditions,
            allow_flightgear_output=False,
        )

    def render(self, mode="human", flightgear_blocking=True):
        """
        Renders the environment.

        :param mode: str, the mode to render with
        :param flightgear_blocking: waits for FlightGear to load before
            returning if True, else returns immediately
        """
        if mode == "human":
            if not self.figure_visualiser:
                self.figure_visualiser = FigureVisualiser(
                    self.sim, self.task.get_props_to_output()  # type: ignore[arg-type]
                )
            self.figure_visualiser.plot(self.sim)  # type: ignore[arg-type]
        elif mode == "flightgear":
            raise ValueError("flightgear rendering is disabled for this class")
        else:
            super().render(mode, flightgear_blocking)
