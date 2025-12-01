"""
Task definitions for flight control environments.

This module defines abstract and concrete task classes for JSBSim-based
reinforcement learning environments, including heading control tasks.
"""
import enum
import math
import random
import types
import warnings
from abc import ABC, abstractmethod
from collections import namedtuple
from typing import Dict, NamedTuple, Optional, Sequence, Tuple, Type

import gymnasium as gym
import numpy as np

import gymnasium_jsbsim.properties as prp
from gymnasium_jsbsim import assessors, constants, rewards, utils
from gymnasium_jsbsim.aircraft import Aircraft
from gymnasium_jsbsim.properties import BoundedProperty, Property
from gymnasium_jsbsim.rewards import RewardStub
from gymnasium_jsbsim.simulation import Simulation


class Task(ABC):
    """
    Interface for Tasks, modules implementing specific environments in JSBSim.

    A task defines its own state space, action space, termination conditions
    and agent_reward function.
    """

    @abstractmethod
    def task_step(
        self, sim: Simulation, action: Sequence[float], sim_steps: int
    ) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Calculates new state, reward and termination.

        :param sim: a Simulation, the simulation from which to extract state
        :param action: sequence of floats, the agent's last action
        :param sim_steps: number of JSBSim integration steps to perform following action
            prior to making observation
        :return: tuple of (observation, reward, terminated, truncated, info) where,
            observation: array, agent's observation of the environment state
            reward: float, the reward for that step
            terminated: bool, True if the episode reached a terminal state
            truncated: bool, True if the episode was truncated (e.g., time limit)
            info: dict, optional, containing diagnostic info for debugging etc.
        """


    @abstractmethod
    def observe_first_state(self, sim: Simulation) -> np.ndarray:
        """
        Initialise any state/controls and get first state observation from reset sim.

        :param sim: Simulation, the environment simulation
        :return: np array, the first state observation of the episode
        """

    @abstractmethod
    def get_initial_conditions(self) -> Optional[Dict[Property, float]]:
        """
        Returns dictionary mapping initial episode conditions to values.

        Episode initial conditions (ICs) are defined by specifying values for
        JSBSim properties, represented by their name (string) in JSBSim.

        JSBSim uses a distinct set of properties for ICs, beginning with 'ic/'
        which differ from property names during the simulation, e.g. "ic/u-fps"
        instead of "velocities/u-fps". See https://jsbsim-team.github.io/jsbsim/

        :return: dict mapping string for each initial condition property to
            initial value, a float, or None to use Env defaults
        """

    @abstractmethod
    def get_state_space(self) -> gym.Space:
        """ Get the task's state Space object """

    @abstractmethod
    def get_action_space(self) -> gym.Space:
        """ Get the task's action Space object """


class FlightTask(Task, ABC):
    """
    Abstract superclass for flight tasks.

    Concrete subclasses should implement the following:
        state_variables attribute: tuple of Propertys, the task's state representation
        action_variables attribute: tuple of Propertys, the task's actions
        get_initial_conditions(): returns dict mapping InitialPropertys to initial values
        _is_terminal(): determines episode termination
        (optional) _new_episode_init(): performs any control input/initialisation on episode reset
        (optional) _update_custom_properties: updates any custom properties in the sim
    """
    base_state_variables = (prp.altitude_sl_ft, prp.pitch_rad, prp.roll_rad,
                            prp.u_fps, prp.v_fps, prp.w_fps,
                            prp.p_radps, prp.q_radps, prp.r_radps,
                            prp.aileron_left, prp.aileron_right, prp.elevator,
                            prp.rudder)
    base_initial_conditions = types.MappingProxyType(  # MappingProxyType makes dict immutable
        {prp.initial_altitude_ft: constants.INITIAL_ALTITUDE_FT,
         prp.initial_terrain_altitude_ft: 0.00000001,
         prp.initial_longitude_geoc_deg: -2.3273,
         prp.initial_latitude_geod_deg: 51.3781  # corresponds to UoBath
         }
    )
    last_agent_reward = Property('reward/last_agent_reward', 'agent reward from step; includes'
                                                             'potential-based shaping reward')
    last_assessment_reward = Property('reward/last_assess_reward', 'assessment reward from step;'
                                                                   'excludes shaping')
    state_variables: Tuple[BoundedProperty, ...]
    action_variables: Tuple[BoundedProperty, ...]
    assessor: assessors.Assessor
    State: Type[NamedTuple]

    def __init__(self, assessor: assessors.Assessor, debug: bool = False) -> None:
        self.last_state = None
        self.assessor = assessor
        self._make_state_class()
        self.debug = debug

    def _make_state_class(self) -> None:
        """ Creates a namedtuple for readable State data """
        # Get list of state property names, containing legal chars only
        legal_attribute_names = [prop.get_legal_name() for prop in
                                 self.state_variables]
        self.State = namedtuple('State', legal_attribute_names)  # pylint: disable=invalid-name

    def task_step(self, sim: Simulation, action: Sequence[float], sim_steps: int) \
            -> Tuple[NamedTuple, float, bool, bool, Dict]:
        # Input actions
        for prop, command in zip(self.action_variables, action):
            sim[prop] = command

        # Run simulation
        for _ in range(sim_steps):
            sim.run()

        self._update_custom_properties(sim)
        state = self.State(*(sim[prop] for prop in self.state_variables))
        terminated = self._is_terminal(sim)
        reward = self.assessor.assess(state, self.last_state, terminated)
        if terminated:
            reward = self._reward_terminal_override(reward, sim)
        if self.debug:
            self._validate_state(state, terminated, action, reward)
        self._store_reward(reward, sim)
        self.last_state = state
        info = {'reward': reward}
        # For now, we don't distinguish between terminated and truncated
        truncated = False

        return state, reward.agent_reward(), terminated, truncated, info

    def _validate_state(self, state, done, action, reward):
        if any(math.isnan(el) for el in state):  # float('nan') in state doesn't work!
            msg = (f'Invalid state encountered!\n'
                   f'State: {state}\n'
                   f'Prev. State: {self.last_state}\n'
                   f'Action: {action}\n'
                   f'Terminal: {done}\n'
                   f'Reward: {reward}')
            warnings.warn(msg, RuntimeWarning)

    def _store_reward(self, reward: rewards.Reward, sim: Simulation):
        sim[self.last_agent_reward] = reward.agent_reward()
        sim[self.last_assessment_reward] = reward.assessment_reward()

    def _update_custom_properties(self, sim: Simulation) -> None:
        """ Calculates any custom properties which change every timestep. """

    @abstractmethod
    def _is_terminal(self, sim: Simulation) -> bool:
        """ Determines whether the current episode should terminate.

        :param sim: the current simulation
        :return: True if the episode should terminate else False
        """

    @abstractmethod
    def _reward_terminal_override(self, reward: rewards.Reward, sim: Simulation) -> bool:
        """
        Determines whether a custom reward is needed, e.g. because
        a terminal condition is met.
        """

    def observe_first_state(self, sim: Simulation) -> np.ndarray:
        self._new_episode_init(sim)
        self._update_custom_properties(sim)
        state = self.State(*(sim[prop] for prop in self.state_variables))
        self.last_state = state
        return state

    def _new_episode_init(self, sim: Simulation) -> None:
        """
        This method is called at the start of every episode. It is used to set
        the value of any controls or environment properties not already defined
        in the task's initial conditions.

        By default it simply starts the aircraft engines.
        """
        sim.start_engines()
        sim.raise_landing_gear()
        self._store_reward(RewardStub(1.0, 1.0), sim)

    @abstractmethod
    def get_initial_conditions(self) -> Dict[Property, float]:
        """Get initial conditions for the task."""

    def get_state_space(self) -> gym.Space:
        state_lows = np.array([state_var.min for state_var in self.state_variables])
        state_highs = np.array([state_var.max for state_var in self.state_variables])
        return gym.spaces.Box(low=state_lows, high=state_highs, dtype=np.float64)

    def get_action_space(self) -> gym.Space:
        action_lows = np.array([act_var.min for act_var in self.action_variables])
        action_highs = np.array([act_var.max for act_var in self.action_variables])
        return gym.spaces.Box(low=action_lows, high=action_highs, dtype=np.float64)


class Shaping(enum.Enum):
    """Reward shaping configuration options."""
    STANDARD = 'STANDARD'
    EXTRA = 'EXTRA'
    EXTRA_SEQUENTIAL = 'EXTRA_SEQUENTIAL'


class HeadingControlTask(FlightTask):
    """
    A task in which the agent must perform steady, level flight maintaining its
    initial heading.
    """
    target_track_deg = BoundedProperty('target/track-deg', 'desired heading [deg]',
                                       prp.heading_deg.min, prp.heading_deg.max)
    track_error_deg = BoundedProperty('error/track-error-deg',
                                      'error to desired track [deg]', -180, 180)
    altitude_error_ft = BoundedProperty('error/altitude-error-ft',
                                        'error to desired altitude [ft]',
                                        prp.altitude_sl_ft.min,
                                        prp.altitude_sl_ft.max)
    action_variables = (prp.aileron_cmd, prp.elevator_cmd, prp.rudder_cmd)

    # pylint: disable=too-many-arguments,too-many-positional-arguments
    def __init__(
        self, shaping_type: Shaping, step_frequency_hz: float,
        aircraft: Aircraft,
        episode_time_s: float = constants.DEFAULT_EPISODE_TIME_S,
        positive_rewards: bool = True
    ):
        """
        Constructor.

        :param step_frequency_hz: the number of agent interaction steps per second
        :param aircraft: the aircraft used in the simulation
        """
        self.max_time_s = episode_time_s
        episode_steps = math.ceil(self.max_time_s * step_frequency_hz)
        self.steps_left = BoundedProperty('info/steps_left', 'steps remaining in episode', 0,
                                          episode_steps)
        self.aircraft = aircraft
        self.extra_state_variables = (self.altitude_error_ft, prp.sideslip_deg,
                                      self.track_error_deg, self.steps_left)
        self.state_variables = FlightTask.base_state_variables + self.extra_state_variables
        self.positive_rewards = positive_rewards
        assessor = self.make_assessor(shaping_type)
        super().__init__(assessor)

    def make_assessor(self, shaping: Shaping) -> assessors.AssessorImpl:
        """Create assessor with base and shaping components based on shaping type."""
        base_components = self._make_base_reward_components()
        shaping_components = ()
        return self._select_assessor(base_components, shaping_components, shaping)

    def _make_base_reward_components(self) -> Tuple[rewards.RewardComponent, ...]:
        base_components = (
            rewards.AsymptoticErrorComponent(name='altitude_error',
                                             prop=self.altitude_error_ft,
                                             state_variables=self.state_variables,
                                             target=0.0,
                                             is_potential_based=False,
                                             scaling_factor=constants.ALTITUDE_SCALING_FT),
            rewards.AsymptoticErrorComponent(name='travel_direction',
                                             prop=self.track_error_deg,
                                             state_variables=self.state_variables,
                                             target=0.0,
                                             is_potential_based=False,
                                             scaling_factor=constants.TRACK_ERROR_SCALING_DEG),
            # Add an airspeed error relative to cruise speed component?
        )
        return base_components

    def _select_assessor(self, base_components: Tuple[rewards.RewardComponent, ...],
                         shaping_components: Tuple[rewards.RewardComponent, ...],
                         shaping: Shaping) -> assessors.AssessorImpl:
        """Select and configure the appropriate assessor based on shaping type."""
        if shaping is Shaping.STANDARD:
            return assessors.AssessorImpl(
                base_components, shaping_components,
                positive_rewards=self.positive_rewards
            )

        wings_level = rewards.AsymptoticErrorComponent(
            name='wings_level',
            prop=prp.roll_rad,
            state_variables=self.state_variables,
            target=0.0,
            is_potential_based=True,
            scaling_factor=constants.ROLL_ERROR_SCALING_RAD
        )
        no_sideslip = rewards.AsymptoticErrorComponent(
            name='no_sideslip',
            prop=prp.sideslip_deg,
            state_variables=self.state_variables,
            target=0.0,
            is_potential_based=True,
            scaling_factor=constants.SIDESLIP_ERROR_SCALING_DEG
        )
        potential_based_components = (wings_level, no_sideslip)

        if shaping is Shaping.EXTRA:
            return assessors.AssessorImpl(
                base_components, potential_based_components,
                positive_rewards=self.positive_rewards
            )
        if shaping is Shaping.EXTRA_SEQUENTIAL:
            _altitude_error, travel_direction = base_components
            # Make the wings_level shaping reward dependent on facing the correct direction
            dependency_map = {wings_level: (travel_direction,)}
            return assessors.ContinuousSequentialAssessor(
                base_components, potential_based_components,
                potential_dependency_map=dependency_map,
                positive_rewards=self.positive_rewards
            )
        return None  # Should never reach here, but satisfies pylint

    def get_initial_conditions(self) -> Dict[Property, float]:
        extra_conditions = {prp.initial_u_fps: self.aircraft.get_cruise_speed_fps(),
                            prp.initial_v_fps: 0,
                            prp.initial_w_fps: 0,
                            prp.initial_p_radps: 0,
                            prp.initial_q_radps: 0,
                            prp.initial_r_radps: 0,
                            prp.initial_roc_fpm: 0,
                            prp.initial_heading_deg: constants.INITIAL_HEADING_DEG,
                            }
        return {**self.base_initial_conditions, **extra_conditions}

    def _update_custom_properties(self, sim: Simulation) -> None:
        self._update_track_error(sim)
        self._update_altitude_error(sim)
        self._decrement_steps_left(sim)

    def _update_track_error(self, sim: Simulation):
        v_north_fps, v_east_fps = sim[prp.v_north_fps], sim[prp.v_east_fps]
        track_deg = prp.Vector2(v_east_fps, v_north_fps).heading_deg()
        target_track_deg = sim[self.target_track_deg]
        error_deg = utils.reduce_reflex_angle_deg(track_deg - target_track_deg)
        sim[self.track_error_deg] = error_deg

    def _update_altitude_error(self, sim: Simulation):
        altitude_ft = sim[prp.altitude_sl_ft]
        target_altitude_ft = self._get_target_altitude()
        error_ft = altitude_ft - target_altitude_ft
        sim[self.altitude_error_ft] = error_ft

    def _decrement_steps_left(self, sim: Simulation):
        sim[self.steps_left] -= 1

    def _is_terminal(self, sim: Simulation) -> bool:
        # Terminate when time >= max, but use math.isclose() for float equality test
        terminal_step = sim[self.steps_left] <= 0
        state_quality = sim[self.last_assessment_reward]
        # TODO: issues if sequential?
        state_out_of_bounds = state_quality < constants.MIN_STATE_QUALITY
        return (terminal_step or state_out_of_bounds or
                self._altitude_out_of_bounds(sim))

    def _altitude_out_of_bounds(self, sim: Simulation) -> bool:
        altitude_error_ft = sim[self.altitude_error_ft]
        return abs(altitude_error_ft) > constants.MAX_ALTITUDE_DEVIATION_FT

    def _get_out_of_bounds_reward(self, sim: Simulation) -> rewards.Reward:
        """
        if aircraft is out of bounds, we give the largest possible negative reward:
        as if this timestep, and every remaining timestep in the episode was -1.
        """
        reward_scalar = (1 + sim[self.steps_left]) * -1.
        return RewardStub(reward_scalar, reward_scalar)

    def _reward_terminal_override(self, reward: rewards.Reward, sim: Simulation) -> rewards.Reward:
        """Override reward on terminal conditions."""
        if self._altitude_out_of_bounds(sim) and not self.positive_rewards:
            # If using negative rewards, need to give a big negative reward on terminal
            return self._get_out_of_bounds_reward(sim)
        return reward

    def _new_episode_init(self, sim: Simulation) -> None:
        super()._new_episode_init(sim)
        sim.set_throttle_mixture_controls(constants.THROTTLE_CMD, constants.MIXTURE_CMD)
        sim[self.steps_left] = self.steps_left.max
        sim[self.target_track_deg] = self._get_target_track()

    def _get_target_track(self) -> float:
        # Use the same, initial heading every episode
        return constants.INITIAL_HEADING_DEG

    def _get_target_altitude(self) -> float:
        return constants.INITIAL_ALTITUDE_FT

    def get_props_to_output(self) -> Tuple:
        """Get properties to output for visualization and monitoring."""
        return (
            prp.u_fps, prp.altitude_sl_ft, self.altitude_error_ft,
            self.target_track_deg, self.track_error_deg, prp.roll_rad,
            prp.sideslip_deg, self.last_agent_reward,
            self.last_assessment_reward, self.steps_left
        )


class TurnHeadingControlTask(HeadingControlTask):
    """
    A task in which the agent must make a turn from a random initial heading,
    and fly level to a random target heading.
    """

    def get_initial_conditions(self) -> [Dict[Property, float]]:
        initial_conditions = super().get_initial_conditions()
        random_heading = random.uniform(prp.heading_deg.min, prp.heading_deg.max)
        initial_conditions[prp.initial_heading_deg] = random_heading
        return initial_conditions

    def _get_target_track(self) -> float:
        """Select a random heading for each episode."""
        return random.uniform(self.target_track_deg.min, self.target_track_deg.max)
