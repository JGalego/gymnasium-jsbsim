# API Reference

## gymnasium_jsbsim

This module registers all combinations of task, aircraft, shaping settings, etc.
with Gymnasium so that they can be instantiated with a gym.make(id) command.

The gymnasium_jsbsim.Envs enum stores all registered environments as members with
their env ID string as value. This allows convenient autocompletion and value safety.

### gymnasium_jsbsim.get_env_id_kwargs_map() → [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[str](https://docs.python.org/3/library/stdtypes.html#str), [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)]

Returns all environment IDs mapped to tuple of (task, aircraft, shaping, flightgear).

### *class* gymnasium_jsbsim.Envs(\*values)

Bases: [`Enum`](https://docs.python.org/3/library/enum.html#enum.Enum)

#### JSBSim_HeadingControlTask_Cessna172P_Shaping_STANDARD_FG_v0 *= 'JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0'*

#### JSBSim_HeadingControlTask_Cessna172P_Shaping_STANDARD_NoFG_v0 *= 'JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0'*

#### JSBSim_HeadingControlTask_Cessna172P_Shaping_EXTRA_FG_v0 *= 'JSBSim-HeadingControlTask-Cessna172P-Shaping.EXTRA-FG-v0'*

#### JSBSim_HeadingControlTask_Cessna172P_Shaping_EXTRA_NoFG_v0 *= 'JSBSim-HeadingControlTask-Cessna172P-Shaping.EXTRA-NoFG-v0'*

#### JSBSim_HeadingControlTask_Cessna172P_Shaping_EXTRA_SEQUENTIAL_FG_v0 *= 'JSBSim-HeadingControlTask-Cessna172P-Shaping.EXTRA_SEQUENTIAL-FG-v0'*

#### JSBSim_HeadingControlTask_Cessna172P_Shaping_EXTRA_SEQUENTIAL_NoFG_v0 *= 'JSBSim-HeadingControlTask-Cessna172P-Shaping.EXTRA_SEQUENTIAL-NoFG-v0'*

#### JSBSim_HeadingControlTask_A320_Shaping_STANDARD_FG_v0 *= 'JSBSim-HeadingControlTask-A320-Shaping.STANDARD-FG-v0'*

#### JSBSim_HeadingControlTask_A320_Shaping_STANDARD_NoFG_v0 *= 'JSBSim-HeadingControlTask-A320-Shaping.STANDARD-NoFG-v0'*

#### JSBSim_HeadingControlTask_A320_Shaping_EXTRA_FG_v0 *= 'JSBSim-HeadingControlTask-A320-Shaping.EXTRA-FG-v0'*

#### JSBSim_HeadingControlTask_A320_Shaping_EXTRA_NoFG_v0 *= 'JSBSim-HeadingControlTask-A320-Shaping.EXTRA-NoFG-v0'*

#### JSBSim_HeadingControlTask_A320_Shaping_EXTRA_SEQUENTIAL_FG_v0 *= 'JSBSim-HeadingControlTask-A320-Shaping.EXTRA_SEQUENTIAL-FG-v0'*

#### JSBSim_HeadingControlTask_A320_Shaping_EXTRA_SEQUENTIAL_NoFG_v0 *= 'JSBSim-HeadingControlTask-A320-Shaping.EXTRA_SEQUENTIAL-NoFG-v0'*

#### JSBSim_HeadingControlTask_F15_Shaping_STANDARD_FG_v0 *= 'JSBSim-HeadingControlTask-F15-Shaping.STANDARD-FG-v0'*

#### JSBSim_HeadingControlTask_F15_Shaping_STANDARD_NoFG_v0 *= 'JSBSim-HeadingControlTask-F15-Shaping.STANDARD-NoFG-v0'*

#### JSBSim_HeadingControlTask_F15_Shaping_EXTRA_FG_v0 *= 'JSBSim-HeadingControlTask-F15-Shaping.EXTRA-FG-v0'*

#### JSBSim_HeadingControlTask_F15_Shaping_EXTRA_NoFG_v0 *= 'JSBSim-HeadingControlTask-F15-Shaping.EXTRA-NoFG-v0'*

#### JSBSim_HeadingControlTask_F15_Shaping_EXTRA_SEQUENTIAL_FG_v0 *= 'JSBSim-HeadingControlTask-F15-Shaping.EXTRA_SEQUENTIAL-FG-v0'*

#### JSBSim_HeadingControlTask_F15_Shaping_EXTRA_SEQUENTIAL_NoFG_v0 *= 'JSBSim-HeadingControlTask-F15-Shaping.EXTRA_SEQUENTIAL-NoFG-v0'*

#### JSBSim_TurnHeadingControlTask_Cessna172P_Shaping_STANDARD_FG_v0 *= 'JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0'*

#### JSBSim_TurnHeadingControlTask_Cessna172P_Shaping_STANDARD_NoFG_v0 *= 'JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0'*

#### JSBSim_TurnHeadingControlTask_Cessna172P_Shaping_EXTRA_FG_v0 *= 'JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.EXTRA-FG-v0'*

#### JSBSim_TurnHeadingControlTask_Cessna172P_Shaping_EXTRA_NoFG_v0 *= 'JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.EXTRA-NoFG-v0'*

#### JSBSim_TurnHeadingControlTask_Cessna172P_Shaping_EXTRA_SEQUENTIAL_FG_v0 *= 'JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.EXTRA_SEQUENTIAL-FG-v0'*

#### JSBSim_TurnHeadingControlTask_Cessna172P_Shaping_EXTRA_SEQUENTIAL_NoFG_v0 *= 'JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.EXTRA_SEQUENTIAL-NoFG-v0'*

#### JSBSim_TurnHeadingControlTask_A320_Shaping_STANDARD_FG_v0 *= 'JSBSim-TurnHeadingControlTask-A320-Shaping.STANDARD-FG-v0'*

#### JSBSim_TurnHeadingControlTask_A320_Shaping_STANDARD_NoFG_v0 *= 'JSBSim-TurnHeadingControlTask-A320-Shaping.STANDARD-NoFG-v0'*

#### JSBSim_TurnHeadingControlTask_A320_Shaping_EXTRA_FG_v0 *= 'JSBSim-TurnHeadingControlTask-A320-Shaping.EXTRA-FG-v0'*

#### JSBSim_TurnHeadingControlTask_A320_Shaping_EXTRA_NoFG_v0 *= 'JSBSim-TurnHeadingControlTask-A320-Shaping.EXTRA-NoFG-v0'*

#### JSBSim_TurnHeadingControlTask_A320_Shaping_EXTRA_SEQUENTIAL_FG_v0 *= 'JSBSim-TurnHeadingControlTask-A320-Shaping.EXTRA_SEQUENTIAL-FG-v0'*

#### JSBSim_TurnHeadingControlTask_A320_Shaping_EXTRA_SEQUENTIAL_NoFG_v0 *= 'JSBSim-TurnHeadingControlTask-A320-Shaping.EXTRA_SEQUENTIAL-NoFG-v0'*

#### JSBSim_TurnHeadingControlTask_F15_Shaping_STANDARD_FG_v0 *= 'JSBSim-TurnHeadingControlTask-F15-Shaping.STANDARD-FG-v0'*

#### JSBSim_TurnHeadingControlTask_F15_Shaping_STANDARD_NoFG_v0 *= 'JSBSim-TurnHeadingControlTask-F15-Shaping.STANDARD-NoFG-v0'*

#### JSBSim_TurnHeadingControlTask_F15_Shaping_EXTRA_FG_v0 *= 'JSBSim-TurnHeadingControlTask-F15-Shaping.EXTRA-FG-v0'*

#### JSBSim_TurnHeadingControlTask_F15_Shaping_EXTRA_NoFG_v0 *= 'JSBSim-TurnHeadingControlTask-F15-Shaping.EXTRA-NoFG-v0'*

#### JSBSim_TurnHeadingControlTask_F15_Shaping_EXTRA_SEQUENTIAL_FG_v0 *= 'JSBSim-TurnHeadingControlTask-F15-Shaping.EXTRA_SEQUENTIAL-FG-v0'*

#### JSBSim_TurnHeadingControlTask_F15_Shaping_EXTRA_SEQUENTIAL_NoFG_v0 *= 'JSBSim-TurnHeadingControlTask-F15-Shaping.EXTRA_SEQUENTIAL-NoFG-v0'*

## gymnasium_jsbsim.environment

Gymnasium environment wrappers for JSBSim flight simulation.

This module provides Env-compliant interfaces to JSBSim for reinforcement
learning (RL) applications with optional FlightGear (FG) visualization support.

### *class* gymnasium_jsbsim.environment.JsbSimEnv(task_type: [Type](https://docs.python.org/3/library/typing.html#typing.Type)[[HeadingControlTask](#gymnasium_jsbsim.tasks.HeadingControlTask)], aircraft: [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft) = ('c172p', 'c172p', 'Cessna172P', 120), agent_interaction_freq: [int](https://docs.python.org/3/library/functions.html#int) = 5, shaping: [Shaping](#gymnasium_jsbsim.tasks.Shaping) = Shaping.STANDARD)

Bases: [`Env`](https://gymnasium.farama.org/api/env/#gymnasium.Env)

Wrapper class for JSBSim based environments.

JsbSimEnv is instantiated with a Task that implements a specific aircraft
control task with its own specific observation/action space and variables
and agent reward calculation.

#### metadata *: [dict](https://docs.python.org/3/library/stdtypes.html#dict)[[str](https://docs.python.org/3/library/stdtypes.html#str), Any]* *= {'render_modes': ['human', 'flightgear']}*

#### \_\_init_\_(task_type: [Type](https://docs.python.org/3/library/typing.html#typing.Type)[[HeadingControlTask](#gymnasium_jsbsim.tasks.HeadingControlTask)], aircraft: [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft) = ('c172p', 'c172p', 'Cessna172P', 120), agent_interaction_freq: [int](https://docs.python.org/3/library/functions.html#int) = 5, shaping: [Shaping](#gymnasium_jsbsim.tasks.Shaping) = Shaping.STANDARD)

Initializes some internal state, but JsbSimEnv.reset() must be called
first before interacting with environment.

* **Parameters:**
  * **task_type** – the Task subclass for the task agent is to perform
  * **aircraft** – the JSBSim aircraft to be used
  * **agent_interaction_freq** – int, how many times per second the agent
    should interact with environment.
  * **shaping** – a HeadingControlTask.Shaping enum, what type of agent_reward
    shaping to use (see HeadingControlTask for options)

#### step(action: ndarray) → [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[ndarray, [float](https://docs.python.org/3/library/functions.html#float), [bool](https://docs.python.org/3/library/functions.html#bool), [bool](https://docs.python.org/3/library/functions.html#bool), [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)]

Run one timestep of the environment’s dynamics.

When end of episode is reached, you are responsible for calling reset()
to reset this environment’s state.

Accepts an action and returns a tuple (observation, reward, terminated, truncated, info).

* **Parameters:**
  **action** – the agent’s action, with same length as action variables.
* **Returns:**
  state: agent’s observation of the current environment
  reward: amount of reward returned after previous action
  terminated: whether the episode reached a terminal state
  truncated: whether the episode was truncated (e.g., time limit)
  info: auxiliary information, e.g. full reward shaping data

#### reset(, seed: [int](https://docs.python.org/3/library/functions.html#int) | [None](https://docs.python.org/3/library/constants.html#None) = None, options: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict) | [None](https://docs.python.org/3/library/constants.html#None) = None)

Resets the environment and returns an initial observation and info.

* **Returns:**
  tuple of (observation, info dict)

#### render(mode='flightgear', flightgear_blocking=True)

Renders the environment.

The set of supported modes varies per environment and some
environments do not support rendering at all.

By convention, if mode is:
- human: render to the current display or terminal and
return nothing. Usually for human consumption.
- rgb_array: return a numpy.ndarray with shape (x, y, 3),
representing RGB values for an x-by-y pixel image, suitable
for turning into a video.
- ansi: return a string (str) or StringIO.StringIO containing a
terminal-style text representation. The text can include newlines
and ANSI escape sequences (e.g. for colors).

Note: Make sure that your class’s metadata ‘render.modes’ key includes
the list of supported modes. It’s recommended to call super() in
implementations to use the functionality of this method.

* **Parameters:**
  * **mode** – str, the mode to render with
  * **flightgear_blocking** – waits for FlightGear to load before
    returning if True, else returns immediately

#### close()

Cleans up this environment’s objects.

Environments automatically close() when garbage collected or when the
program exits.

#### seed(seed=None)

Sets the seed for this environment’s random number generator(s).

#### NOTE
Some environments use multiple pseudorandom number generators.
We want to capture all such seeds used in order to ensure that
there aren’t accidental correlations between multiple generators.

### *class* gymnasium_jsbsim.environment.NoFGJsbSimEnv(task_type: [Type](https://docs.python.org/3/library/typing.html#typing.Type)[[HeadingControlTask](#gymnasium_jsbsim.tasks.HeadingControlTask)], aircraft: [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft) = ('c172p', 'c172p', 'Cessna172P', 120), agent_interaction_freq: [int](https://docs.python.org/3/library/functions.html#int) = 5, shaping: [Shaping](#gymnasium_jsbsim.tasks.Shaping) = Shaping.STANDARD)

Bases: [`JsbSimEnv`](#gymnasium_jsbsim.environment.JsbSimEnv)

Wrapper class for JSBSim based environments with FlightGear disabled.

This class exists to be used for training agents where visualisation is not
required. Otherwise, restrictions in JSBSim output initialisation cause it
to open a new socket for every single episode.

#### metadata *: [dict](https://docs.python.org/3/library/stdtypes.html#dict)[[str](https://docs.python.org/3/library/stdtypes.html#str), Any]* *= {'render_modes': ['human']}*

#### \_\_init_\_(task_type: [Type](https://docs.python.org/3/library/typing.html#typing.Type)[[HeadingControlTask](#gymnasium_jsbsim.tasks.HeadingControlTask)], aircraft: [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft) = ('c172p', 'c172p', 'Cessna172P', 120), agent_interaction_freq: [int](https://docs.python.org/3/library/functions.html#int) = 5, shaping: [Shaping](#gymnasium_jsbsim.tasks.Shaping) = Shaping.STANDARD)

Initializes some internal state, but JsbSimEnv.reset() must be called
first before interacting with environment.

* **Parameters:**
  * **task_type** – the Task subclass for the task agent is to perform
  * **aircraft** – the JSBSim aircraft to be used
  * **agent_interaction_freq** – int, how many times per second the agent
    should interact with environment.
  * **shaping** – a HeadingControlTask.Shaping enum, what type of agent_reward
    shaping to use (see HeadingControlTask for options)

#### step(action: ndarray) → [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[ndarray, [float](https://docs.python.org/3/library/functions.html#float), [bool](https://docs.python.org/3/library/functions.html#bool), [bool](https://docs.python.org/3/library/functions.html#bool), [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)]

Run one timestep of the environment’s dynamics.

When end of episode is reached, you are responsible for calling reset()
to reset this environment’s state.

Accepts an action and returns a tuple (observation, reward, terminated, truncated, info).

* **Parameters:**
  **action** – the agent’s action, with same length as action variables.
* **Returns:**
  state: agent’s observation of the current environment
  reward: amount of reward returned after previous action
  terminated: whether the episode reached a terminal state
  truncated: whether the episode was truncated (e.g., time limit)
  info: auxiliary information, e.g. full reward shaping data

#### reset(, seed: [int](https://docs.python.org/3/library/functions.html#int) | [None](https://docs.python.org/3/library/constants.html#None) = None, options: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict) | [None](https://docs.python.org/3/library/constants.html#None) = None)

Resets the environment and returns an initial observation and info.

* **Returns:**
  tuple of (observation, info dict)

#### render(mode='human', flightgear_blocking=True)

Renders the environment.

* **Parameters:**
  * **mode** – str, the mode to render with
  * **flightgear_blocking** – waits for FlightGear to load before
    returning if True, else returns immediately

## gymnasium_jsbsim.tasks

Task definitions for flight control environments.

This module defines abstract and concrete task classes for JSBSim-based
reinforcement learning environments, including heading control tasks.

### *class* gymnasium_jsbsim.tasks.Task

Bases: [`ABC`](https://docs.python.org/3/library/abc.html#abc.ABC)

Interface class for JSBSim tasks.

A task defines its own state space, action space, termination conditions
and agent_reward function.

#### *abstractmethod* task_step(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation), action: [Sequence](https://docs.python.org/3/library/typing.html#typing.Sequence)[[float](https://docs.python.org/3/library/functions.html#float)], sim_steps: [int](https://docs.python.org/3/library/functions.html#int)) → [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[ndarray, [float](https://docs.python.org/3/library/functions.html#float), [bool](https://docs.python.org/3/library/functions.html#bool), [bool](https://docs.python.org/3/library/functions.html#bool), [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)]

Calculates new state, reward and termination.

* **Parameters:**
  * **sim** – a Simulation, the simulation from which to extract state
  * **action** – sequence of floats, the agent’s last action
  * **sim_steps** – number of JSBSim integration steps to perform following action
    prior to making observation
* **Returns:**
  tuple of (observation, reward, terminated, truncated, info) where,
  observation: array, agent’s observation of the environment state
  reward: float, the reward for that step
  terminated: bool, True if the episode reached a terminal state
  truncated: bool, True if the episode was truncated e.g. time limit
  info: dict, optional, containing diagnostic info for debugging etc.

#### *abstractmethod* observe_first_state(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation)) → ndarray

Initialise any state/controls and get first state observation from reset sim.

* **Parameters:**
  **sim** – Simulation, the environment simulation
* **Returns:**
  np.ndarray, the first state observation of the episode

#### *abstractmethod* get_initial_conditions() → [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[Property](#gymnasium_jsbsim.properties.Property), [float](https://docs.python.org/3/library/functions.html#float)] | [None](https://docs.python.org/3/library/constants.html#None)

Returns dictionary mapping initial episode conditions to values.

Episode initial conditions (ICs) are defined by specifying values for
JSBSim properties, represented by their name (string) in JSBSim.

JSBSim uses a distinct set of properties for ICs, beginning with ic/
which differ from property names during the simulation, e.g. “ic/u-fps”
instead of “velocities/u-fps”. See [https://jsbsim-team.github.io/jsbsim/](https://jsbsim-team.github.io/jsbsim/)

* **Returns:**
  dict mapping string for each initial condition property to
  initial value, a float, or None to use Env defaults

#### *abstractmethod* get_state_space() → [Space](https://gymnasium.farama.org/api/spaces/#gymnasium.spaces.Space)

Get the task state Space object

#### *abstractmethod* get_action_space() → [Space](https://gymnasium.farama.org/api/spaces/#gymnasium.spaces.Space)

Get the task action Space object

### *class* gymnasium_jsbsim.tasks.FlightTask(assessor: [Assessor](#gymnasium_jsbsim.assessors.Assessor), debug: [bool](https://docs.python.org/3/library/functions.html#bool) = False)

Bases: [`Task`](#gymnasium_jsbsim.tasks.Task), [`ABC`](https://docs.python.org/3/library/abc.html#abc.ABC)

Abstract superclass for flight tasks.

Concrete subclasses should implement the following:
: state_variables attribute: tuple of Propertys, the task’s state representation
  action_variables attribute: tuple of Propertys, the task’s actions
  get_initial_conditions(): returns dict mapping InitialPropertys to initial values
  \_is_terminal(): determines episode termination
  (optional) \_new_episode_init(): performs any control input/initialisation on episode reset
  (optional) \_update_custom_properties: updates any custom properties in the sim

#### base_state_variables *= (('position/h-sl-ft', 'altitude above mean sea level [ft]', -1400, 85000), ('attitude/pitch-rad', 'pitch [rad]', -1.5707963267948966, 1.5707963267948966), ('attitude/roll-rad', 'roll [rad]', -3.141592653589793, 3.141592653589793), ('velocities/u-fps', 'body frame x-axis velocity [ft/s]', -2200, 2200), ('velocities/v-fps', 'body frame y-axis velocity [ft/s]', -2200, 2200), ('velocities/w-fps', 'body frame z-axis velocity [ft/s]', -2200, 2200), ('velocities/p-rad_sec', 'roll rate [rad/s]', -6.283185307179586, 6.283185307179586), ('velocities/q-rad_sec', 'pitch rate [rad/s]', -6.283185307179586, 6.283185307179586), ('velocities/r-rad_sec', 'yaw rate [rad/s]', -6.283185307179586, 6.283185307179586), ('fcs/left-aileron-pos-norm', 'left aileron position, normalised', -1, 1), ('fcs/right-aileron-pos-norm', 'right aileron position, normalised', -1, 1), ('fcs/elevator-pos-norm', 'elevator position, normalised', -1, 1), ('fcs/rudder-pos-norm', 'rudder position, normalised', -1, 1))*

#### base_initial_conditions *= mappingproxy({Property(name='ic/h-sl-ft', description='initial altitude MSL [ft]'): 5000, Property(name='ic/terrain-elevation-ft', description='initial terrain alt [ft]'): 1e-08, Property(name='ic/long-gc-deg', description='initial geocentric longitude [deg]'): -2.3273, Property(name='ic/lat-geod-deg', description='initial geodesic latitude [deg]'): 51.3781})*

#### last_agent_reward *= ('reward/last_agent_reward', 'agent reward from step; includes potential-based shaping reward')*

#### last_assessment_reward *= ('reward/last_assess_reward', 'assessment reward from step; excludes shaping')*

#### state_variables *: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), ...]*

#### action_variables *: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), ...]*

#### State *: [type](https://docs.python.org/3/library/functions.html#type)*

#### \_\_init_\_(assessor: [Assessor](#gymnasium_jsbsim.assessors.Assessor), debug: [bool](https://docs.python.org/3/library/functions.html#bool) = False) → [None](https://docs.python.org/3/library/constants.html#None)

#### assessor *: [Assessor](#gymnasium_jsbsim.assessors.Assessor)*

#### task_step(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation), action: [Sequence](https://docs.python.org/3/library/typing.html#typing.Sequence)[[float](https://docs.python.org/3/library/functions.html#float)], sim_steps: [int](https://docs.python.org/3/library/functions.html#int)) → [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[ndarray, [float](https://docs.python.org/3/library/functions.html#float), [bool](https://docs.python.org/3/library/functions.html#bool), [bool](https://docs.python.org/3/library/functions.html#bool), [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)]

Calculates new state, reward and termination.

* **Parameters:**
  * **sim** – a Simulation, the simulation from which to extract state
  * **action** – sequence of floats, the agent’s last action
  * **sim_steps** – number of JSBSim integration steps to perform following action
    prior to making observation
* **Returns:**
  tuple of (observation, reward, terminated, truncated, info) where,
  observation: array, agent’s observation of the environment state
  reward: float, the reward for that step
  terminated: bool, True if the episode reached a terminal state
  truncated: bool, True if the episode was truncated e.g. time limit
  info: dict, optional, containing diagnostic info for debugging etc.

#### observe_first_state(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation)) → ndarray

Initialise any state/controls and get first state observation from reset sim.

* **Parameters:**
  **sim** – Simulation, the environment simulation
* **Returns:**
  np.ndarray, the first state observation of the episode

#### *abstractmethod* get_initial_conditions() → [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[Property](#gymnasium_jsbsim.properties.Property), [float](https://docs.python.org/3/library/functions.html#float)]

Get initial conditions for the task.

#### get_state_space() → [Space](https://gymnasium.farama.org/api/spaces/#gymnasium.spaces.Space)

Get the task state Space object.

* **Returns:**
  gym.Space representing the task state space

#### get_action_space() → [Space](https://gymnasium.farama.org/api/spaces/#gymnasium.spaces.Space)

Get the task action Space object.

* **Returns:**
  gym.Space representing the task action space

### *class* gymnasium_jsbsim.tasks.Shaping(\*values)

Bases: [`Enum`](https://docs.python.org/3/library/enum.html#enum.Enum)

Reward shaping configuration options.

#### STANDARD *= 'STANDARD'*

#### EXTRA *= 'EXTRA'*

#### EXTRA_SEQUENTIAL *= 'EXTRA_SEQUENTIAL'*

### *class* gymnasium_jsbsim.tasks.HeadingControlTask(shaping_type: [Shaping](#gymnasium_jsbsim.tasks.Shaping), step_frequency_hz: [float](https://docs.python.org/3/library/functions.html#float), aircraft: [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft), episode_time_s: [float](https://docs.python.org/3/library/functions.html#float) = 60.0, positive_rewards: [bool](https://docs.python.org/3/library/functions.html#bool) = True)

Bases: [`FlightTask`](#gymnasium_jsbsim.tasks.FlightTask)

A task in which the agent must perform steady, level flight maintaining its
initial heading.

#### target_track_deg *= ('target/track-deg', 'desired heading [deg]', 0, 360)*

#### track_error_deg *= ('error/track-error-deg', 'error to desired track [deg]', -180, 180)*

#### altitude_error_ft *= ('error/altitude-error-ft', 'error to desired altitude [ft]', -1400, 85000)*

#### action_variables *: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), ...]* *= (('fcs/aileron-cmd-norm', 'aileron commanded position, normalised', -1.0, 1.0), ('fcs/elevator-cmd-norm', 'elevator commanded position, normalised', -1.0, 1.0), ('fcs/rudder-cmd-norm', 'rudder commanded position, normalised', -1.0, 1.0))*

#### \_\_init_\_(shaping_type: [Shaping](#gymnasium_jsbsim.tasks.Shaping), step_frequency_hz: [float](https://docs.python.org/3/library/functions.html#float), aircraft: [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft), episode_time_s: [float](https://docs.python.org/3/library/functions.html#float) = 60.0, positive_rewards: [bool](https://docs.python.org/3/library/functions.html#bool) = True)

Initializes a HeadingControlTask.

* **Parameters:**
  * **step_frequency_hz** – the number of agent interaction steps per second
  * **aircraft** – the aircraft used in the simulation

#### make_assessor(shaping: [Shaping](#gymnasium_jsbsim.tasks.Shaping)) → [AssessorImpl](#gymnasium_jsbsim.assessors.AssessorImpl)

Create assessor with base and shaping components based on shaping type.

* **Parameters:**
  **shaping** – the shaping configuration
* **Returns:**
  the configured assessor

#### get_initial_conditions() → [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[Property](#gymnasium_jsbsim.properties.Property), [float](https://docs.python.org/3/library/functions.html#float)]

Get initial conditions for the task.
:return: dict mapping initial condition Propertys to values

#### get_props_to_output() → [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)

Get properties to output for visualization and monitoring.
:return: tuple of properties to output

### *class* gymnasium_jsbsim.tasks.TurnHeadingControlTask(shaping_type: [Shaping](#gymnasium_jsbsim.tasks.Shaping), step_frequency_hz: [float](https://docs.python.org/3/library/functions.html#float), aircraft: [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft), episode_time_s: [float](https://docs.python.org/3/library/functions.html#float) = 60.0, positive_rewards: [bool](https://docs.python.org/3/library/functions.html#bool) = True)

Bases: [`HeadingControlTask`](#gymnasium_jsbsim.tasks.HeadingControlTask)

A task in which the agent must make a turn from a random initial heading,
and fly level to a random target heading.

#### get_initial_conditions() → [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[Property](#gymnasium_jsbsim.properties.Property), [float](https://docs.python.org/3/library/functions.html#float)]

Get initial conditions for the task.
:return: dict mapping initial condition Propertys to values

## gymnasium_jsbsim.aircraft

Aircraft module defining aircraft configurations for JSBSim simulation.

### *class* gymnasium_jsbsim.aircraft.Aircraft(jsbsim_id, flightgear_id, name, cruise_speed_kts)

Bases: [`Aircraft`](#gymnasium_jsbsim.aircraft.Aircraft)

Aircraft configuration containing identifiers and performance characteristics.

#### get_max_distance_m(episode_time_s: [float](https://docs.python.org/3/library/functions.html#float)) → [float](https://docs.python.org/3/library/functions.html#float)

Estimates the maximum distance this aircraft can travel in an episode.

* **Parameters:**
  **episode_time_s** – Episode duration in seconds
* **Returns:**
  Maximum distance in meters

#### get_cruise_speed_fps() → [float](https://docs.python.org/3/library/functions.html#float)

Get the cruise speed in feet per second.

* **Returns:**
  Cruise speed in feet per second

## gymnasium_jsbsim.simulation

Wrapper for JSBSim flight dynamics simulator.

### *class* gymnasium_jsbsim.simulation.Simulation(sim_frequency_hz: [float](https://docs.python.org/3/library/functions.html#float) = 60.0, aircraft: [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft) = ('c172p', 'c172p', 'Cessna172P', 120), init_conditions: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[Property](#gymnasium_jsbsim.properties.Property), [float](https://docs.python.org/3/library/functions.html#float)] | [None](https://docs.python.org/3/library/constants.html#None) = None, allow_flightgear_output: [bool](https://docs.python.org/3/library/functions.html#bool) = True, root_dir: [str](https://docs.python.org/3/library/stdtypes.html#str) | [None](https://docs.python.org/3/library/constants.html#None) = None)

Bases: [`object`](https://docs.python.org/3/library/functions.html#object)

Wrapper class for JSBSim flight dynamics simulator.

#### \_\_init_\_(sim_frequency_hz: [float](https://docs.python.org/3/library/functions.html#float) = 60.0, aircraft: [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft) = ('c172p', 'c172p', 'Cessna172P', 120), init_conditions: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[Property](#gymnasium_jsbsim.properties.Property), [float](https://docs.python.org/3/library/functions.html#float)] | [None](https://docs.python.org/3/library/constants.html#None) = None, allow_flightgear_output: [bool](https://docs.python.org/3/library/functions.html#bool) = True, root_dir: [str](https://docs.python.org/3/library/stdtypes.html#str) | [None](https://docs.python.org/3/library/constants.html#None) = None)

Creates and initializes a JSBSim simulation instance.

* **Parameters:**
  * **sim_frequency_hz** – the JSBSim integration frequency in Hz.
  * **aircraft** – Aircraft instance to be loaded.
  * **init_conditions** – dict mapping properties to their initial values.
    Defaults to None, causing a default set of initial props to be used.
  * **allow_flightgear_output** – bool, loads a config file instructing
    JSBSim to connect to an output socket if True.
  * **root_dir** – JSBSim root directory path
* **Raises:**
  [**RuntimeError**](https://docs.python.org/3/library/exceptions.html#RuntimeError) – if JSBSim fails to initialize or load the model

#### \_\_getitem_\_(prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty) | [Property](#gymnasium_jsbsim.properties.Property)) → [float](https://docs.python.org/3/library/functions.html#float)

Gets simulation property value.

Properties are identified by strings. A list can be found in the JSBSim
reference manual, launching JSBSim with ‘–catalog’ command line arg or
calling FGFDMExec.get_property_catalog().

* **Parameters:**
  **prop** – BoundedProperty, the property to be retrieved
* **Returns:**
  float

#### \_\_setitem_\_(prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty) | [Property](#gymnasium_jsbsim.properties.Property), value) → [None](https://docs.python.org/3/library/constants.html#None)

Sets simulation property to specified value.

Properties are identified by strings. A list can be found in the JSBSim
reference manual, launching JSBSim with ‘–catalog’ command line arg or
calling FGFDMExec.get_property_catalog().

Warning: JSBSim will create new properties if the specified one exists.
If the property you are setting is read-only in JSBSim the operation
will silently fail.

* **Parameters:**
  * **prop** – BoundedProperty, the property to be retrieved
  * **value** – object?, the value to be set

#### load_model(model_name: [str](https://docs.python.org/3/library/stdtypes.html#str)) → [None](https://docs.python.org/3/library/constants.html#None)

Loads the specified aircraft config into the simulation.

The root JSBSim directory aircraft folder is searched for the aircraft
XML config file.

* **Parameters:**
  **model_name** – string, the aircraft name

#### get_aircraft() → [Aircraft](#gymnasium_jsbsim.aircraft.Aircraft)

Gets the Aircraft this sim was initialised with.

#### get_loaded_model_name() → [str](https://docs.python.org/3/library/stdtypes.html#str) | [None](https://docs.python.org/3/library/constants.html#None)

Gets the name of the aircraft model currently loaded in JSBSim.

* **Returns:**
  string, the name of the aircraft model if one is loaded, or
  None if no model is loaded.

#### get_sim_time() → [float](https://docs.python.org/3/library/functions.html#float)

Gets the simulation time from JSBSim, a float.

* **Returns:**
  Current simulation time in seconds

#### initialise(dt: [float](https://docs.python.org/3/library/functions.html#float), model_name: [str](https://docs.python.org/3/library/stdtypes.html#str), init_conditions: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[Property](#gymnasium_jsbsim.properties.Property), [float](https://docs.python.org/3/library/functions.html#float)] | [None](https://docs.python.org/3/library/constants.html#None) = None) → [None](https://docs.python.org/3/library/constants.html#None)

Initialises JSBSim with the specified aircraft and initial conditions.

JSBSim creates an InitialConditions object internally when given an
XML config file. This method either loads a basic set of ICs, or
can be passed a dictionary with ICs. In the latter case a minimal IC
XML file is loaded, and then the dictionary values are fed in.

* **Parameters:**
  * **dt** – float, the JSBSim integration timestep in seconds
  * **model_name** – string, name of aircraft to be loaded
  * **init_conditions** – dict mapping properties to their initial values

#### set_custom_initial_conditions(init_conditions: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[Property](#gymnasium_jsbsim.properties.Property), [float](https://docs.python.org/3/library/functions.html#float)] | [None](https://docs.python.org/3/library/constants.html#None) = None) → [None](https://docs.python.org/3/library/constants.html#None)

Set custom initial conditions in the simulation.

* **Parameters:**
  **init_conditions** – Dictionary mapping properties to their initial values

#### reinitialise(init_conditions: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[Property](#gymnasium_jsbsim.properties.Property), [float](https://docs.python.org/3/library/functions.html#float)] | [None](https://docs.python.org/3/library/constants.html#None) = None) → [None](https://docs.python.org/3/library/constants.html#None)

Reinitialises the JSBSim simulation to the initial conditions.

The same aircraft and other settings are kept loaded in JSBSim. If a
dict of ICs is provided, JSBSim is initialised using these, else the
last specified ICs are used.

* **Parameters:**
  **init_conditions** – dict mapping properties to their initial values

#### run() → [bool](https://docs.python.org/3/library/functions.html#bool)

Advances the simulation by one timestep.

JSBSim monitors the simulation and detects whether it thinks it should
end, e.g. because a simulation time was specified.

* **Returns:**
  bool, False if the simulation has met JSBSim termination
  criteria else True.

#### enable_flightgear_output()

Enables output from the FlightGear simulator.

#### disable_flightgear_output()

Disables output from the FlightGear simulator.

#### close()

Closes the simulation and any plots.

#### set_simulation_time_factor(time_factor)

Sets the simulation speed relative to realtime.

The simulation runs at realtime for time_factor=1. It runs at double
speed for time_factor=2, and half speed for 0.5.

* **Parameters:**
  **time_factor** – int or float, nonzero, sim speed relative to realtime
  if None, the simulation is run at maximum computational speed

#### start_engines()

Starts all aircraft engines.

#### set_throttle_mixture_controls(throttle_cmd: [float](https://docs.python.org/3/library/functions.html#float), mixture_cmd: [float](https://docs.python.org/3/library/functions.html#float))

Sets the throttle and mixture commands for the aircraft.

If an aircraft is multi-engine and has multiple throttle_cmd and mixture_cmd
controls, sets all of them. Currently only supports up to two throttles/mixtures.

#### raise_landing_gear()

Raises the aircraft landing gear.

## gymnasium_jsbsim.rewards

Reward calculation and component classes for reinforcement learning.

This module provides classes for computing rewards in RL environments,
including support for reward shaping and potential-based rewards.

### *class* gymnasium_jsbsim.rewards.Reward(base_reward_elements: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple), shaping_reward_elements: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple))

Bases: [`object`](https://docs.python.org/3/library/functions.html#object)

Immutable class representing the reward obtained at a timestep.

We decompose rewards into tuples of component values, reflecting contributions
from different goals. Separate tuples are maintained for the assessment (non-shaping)
components and the shaping components. It is intended that the

Scalar reward values are retrieved by calling agent_reward() or assessment_reward().
The scalar value is the mean of the components.

#### \_\_init_\_(base_reward_elements: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple), shaping_reward_elements: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple))

#### agent_reward() → [float](https://docs.python.org/3/library/functions.html#float)

Returns scalar reward value by taking the mean of all reward elements.

* **Returns:**
  Mean of all reward components (base + shaping)

#### assessment_reward() → [float](https://docs.python.org/3/library/functions.html#float)

Returns scalar non-shaping reward by taking mean of base reward elements.

* **Returns:**
  Mean of base (non-shaping) reward components

#### is_shaping()

Checks if this reward includes shaping components.

* **Returns:**
  True if shaping components are present, False otherwise

### *class* gymnasium_jsbsim.rewards.RewardComponent

Bases: [`ABC`](https://docs.python.org/3/library/abc.html#abc.ABC)

Interface for RewardComponent, an object which calculates one component value of a Reward.

#### *abstractmethod* calculate(state: tasks.FlightTask.State, last_state: tasks.FlightTask.State, is_terminal: [bool](https://docs.python.org/3/library/functions.html#bool)) → [float](https://docs.python.org/3/library/functions.html#float)

Calculate the reward component value.

#### *abstractmethod* get_name() → [str](https://docs.python.org/3/library/stdtypes.html#str)

Get the name of this reward component.

#### *abstractmethod* get_potential(state: tasks.FlightTask.State, is_terminal) → [float](https://docs.python.org/3/library/functions.html#float)

Get the potential value for the given state.

#### *abstractmethod* is_potential_difference_based() → [bool](https://docs.python.org/3/library/functions.html#bool)

Check if this component uses potential-based difference calculation.

### *class* gymnasium_jsbsim.rewards.NormalisedComponent(name: [str](https://docs.python.org/3/library/stdtypes.html#str), prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), state_variables: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), ...], target: [int](https://docs.python.org/3/library/functions.html#int) | [float](https://docs.python.org/3/library/functions.html#float) | [Property](#gymnasium_jsbsim.properties.Property) | [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), potential_difference_based: [bool](https://docs.python.org/3/library/functions.html#bool))

Bases: [`RewardComponent`](#gymnasium_jsbsim.rewards.RewardComponent), [`ABC`](https://docs.python.org/3/library/abc.html#abc.ABC)

Base implementation of RewardComponent for components that normalise errors.

All potentials of subclasses should be normalised in [0.0, 1.0]

#### \_\_init_\_(name: [str](https://docs.python.org/3/library/stdtypes.html#str), prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), state_variables: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), ...], target: [int](https://docs.python.org/3/library/functions.html#int) | [float](https://docs.python.org/3/library/functions.html#float) | [Property](#gymnasium_jsbsim.properties.Property) | [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), potential_difference_based: [bool](https://docs.python.org/3/library/functions.html#bool))

Initializes NormalisedComponent with property, target, and potential type.

* **Parameters:**
  * **name** – the uniquely identifying name of this component
  * **prop** – the BoundedProperty for which a value will be retrieved
    from the State
  * **state_variables** – the state variables corresponding to each State element
    that this component will be passed.
  * **potential_difference_based** – True if reward is based on a potential difference
    between prev_state and state (AKA potential-based shaping reward) else
    False (and reward depends only on the potential of current state).

#### calculate(state: tasks.FlightTask.State, last_state: tasks.FlightTask.State, is_terminal: [bool](https://docs.python.org/3/library/functions.html#bool))

Calculates the value of this RewardComponent.

If this component is potential difference based, its value is the
difference in potentials between last_state and state. Otherwise its
value is the potential of state.

#### is_constant_target()

Checks if the target value is a constant.

#### get_name() → [str](https://docs.python.org/3/library/stdtypes.html#str)

Returns the name of this RewardComponent.

#### is_potential_difference_based() → [bool](https://docs.python.org/3/library/functions.html#bool)

Checks if this component uses potential-based difference calculation.

### *class* gymnasium_jsbsim.rewards.ErrorComponent(name: [str](https://docs.python.org/3/library/stdtypes.html#str), prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), state_variables: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), ...], target: [int](https://docs.python.org/3/library/functions.html#int) | [float](https://docs.python.org/3/library/functions.html#float) | [Property](#gymnasium_jsbsim.properties.Property) | [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), potential_difference_based: [bool](https://docs.python.org/3/library/functions.html#bool))

Bases: [`NormalisedComponent`](#gymnasium_jsbsim.rewards.NormalisedComponent), [`ABC`](https://docs.python.org/3/library/abc.html#abc.ABC)

Calculates rewards based on a normalised error complement from a target value.

Normalising an error takes some absolute difference abs(value - target) and
transforms it to the interval [0,1], where 1.0 is no error and 0.0 is inf error.

#### get_potential(state: tasks.FlightTask.State, is_terminal) → [float](https://docs.python.org/3/library/functions.html#float)

Calculates the ‘goodness’ of a State given we want the compare_property
to be some target_value. The target value may be a constant or
retrieved from another property in the state.

The ‘goodness’ of the state is given in the interval [-1,0], where 0
corresponds to zero error, and -1 corresponds to inf error.

### *class* gymnasium_jsbsim.rewards.AsymptoticErrorComponent(name: [str](https://docs.python.org/3/library/stdtypes.html#str), prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), state_variables: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), ...], target: [int](https://docs.python.org/3/library/functions.html#int) | [float](https://docs.python.org/3/library/functions.html#float) | [Property](#gymnasium_jsbsim.properties.Property) | [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), is_potential_based: [bool](https://docs.python.org/3/library/functions.html#bool), scaling_factor: [float](https://docs.python.org/3/library/functions.html#float) | [int](https://docs.python.org/3/library/functions.html#int))

Bases: [`ErrorComponent`](#gymnasium_jsbsim.rewards.ErrorComponent)

A reward component which gives a negative reward that asymptotically approaches -1
as the error to the desired value approaches +inf. This is convenient for not having
to worry about the bounds on the absolute error value.

#### \_\_init_\_(name: [str](https://docs.python.org/3/library/stdtypes.html#str), prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), state_variables: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), ...], target: [int](https://docs.python.org/3/library/functions.html#int) | [float](https://docs.python.org/3/library/functions.html#float) | [Property](#gymnasium_jsbsim.properties.Property) | [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), is_potential_based: [bool](https://docs.python.org/3/library/functions.html#bool), scaling_factor: [float](https://docs.python.org/3/library/functions.html#float) | [int](https://docs.python.org/3/library/functions.html#int))

Initializes AsymptoticErrorComponent with scaling factor.

* **Parameters:**
  **scaling_factor** – the property value is scaled down by this amount.
  Shaping potential is at 0.5 when the error equals this factor.

### *class* gymnasium_jsbsim.rewards.AngularAsymptoticErrorComponent(name: [str](https://docs.python.org/3/library/stdtypes.html#str), prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), state_variables: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), ...], target: [int](https://docs.python.org/3/library/functions.html#int) | [float](https://docs.python.org/3/library/functions.html#float) | [Property](#gymnasium_jsbsim.properties.Property) | [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), is_potential_based: [bool](https://docs.python.org/3/library/functions.html#bool), scaling_factor: [float](https://docs.python.org/3/library/functions.html#float) | [int](https://docs.python.org/3/library/functions.html#int))

Bases: [`AsymptoticErrorComponent`](#gymnasium_jsbsim.rewards.AsymptoticErrorComponent)

A potential-based shaping reward component.

Potential is based asymptotically on the  size of the error between a
property of interest and its target. The error can be unbounded in
magnitude.

Values must be in units of degrees. Errors are reduced to the interval
(-180, 180] before processing.

### *class* gymnasium_jsbsim.rewards.LinearErrorComponent(name: [str](https://docs.python.org/3/library/stdtypes.html#str), prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), state_variables: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty)], target: [int](https://docs.python.org/3/library/functions.html#int) | [float](https://docs.python.org/3/library/functions.html#float) | [Property](#gymnasium_jsbsim.properties.Property) | [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), is_potential_based: [bool](https://docs.python.org/3/library/functions.html#bool), scaling_factor: [float](https://docs.python.org/3/library/functions.html#float) | [int](https://docs.python.org/3/library/functions.html#int))

Bases: [`ErrorComponent`](#gymnasium_jsbsim.rewards.ErrorComponent)

A potential-based shaping reward component.

Potential is based linearly on the size of the error between a property of
interest and its target. The error must be in the interval [0, scaling_factor].

#### \_\_init_\_(name: [str](https://docs.python.org/3/library/stdtypes.html#str), prop: [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), state_variables: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty)], target: [int](https://docs.python.org/3/library/functions.html#int) | [float](https://docs.python.org/3/library/functions.html#float) | [Property](#gymnasium_jsbsim.properties.Property) | [BoundedProperty](#gymnasium_jsbsim.properties.BoundedProperty), is_potential_based: [bool](https://docs.python.org/3/library/functions.html#bool), scaling_factor: [float](https://docs.python.org/3/library/functions.html#float) | [int](https://docs.python.org/3/library/functions.html#int))

Constructor.

* **Parameters:**
  **scaling_factor** – the max size of the difference between prop and
  target. Minimum potential (0.0) occurs when error is
  max_error_size or greater.

### gymnasium_jsbsim.rewards.normalise_error_asymptotic(absolute_error: [float](https://docs.python.org/3/library/functions.html#float), scaling_factor: [float](https://docs.python.org/3/library/functions.html#float)) → [float](https://docs.python.org/3/library/functions.html#float)

Given an error in the interval [0, +inf], returns a normalised error in [0, 1]

The normalised error asymptotically approaches 1 as absolute_error -> +inf.

The parameter scaling_factor is used to scale for magnitude.
When absolute_error == scaling_factor, the normalised error is equal to 0.5

### gymnasium_jsbsim.rewards.normalise_error_linear(absolute_error: [float](https://docs.python.org/3/library/functions.html#float), max_error: [float](https://docs.python.org/3/library/functions.html#float)) → [float](https://docs.python.org/3/library/functions.html#float)

Given an absolute error in [0, max_error], linearly normalises error in [0, 1]

If absolute_error exceeds max_error, it is capped back to max_error

### *class* gymnasium_jsbsim.rewards.RewardStub(agent_reward_value: [float](https://docs.python.org/3/library/functions.html#float), assessment_reward_value: [float](https://docs.python.org/3/library/functions.html#float))

Bases: [`Reward`](#gymnasium_jsbsim.rewards.Reward)

Test stub for Reward class.

#### \_\_init_\_(agent_reward_value: [float](https://docs.python.org/3/library/functions.html#float), assessment_reward_value: [float](https://docs.python.org/3/library/functions.html#float))

#### agent_reward() → [float](https://docs.python.org/3/library/functions.html#float)

Returns scalar reward value by taking the mean of all reward elements.

* **Returns:**
  Mean of all reward components (base + shaping)

#### assessment_reward() → [float](https://docs.python.org/3/library/functions.html#float)

Returns scalar non-shaping reward by taking mean of base reward elements.

* **Returns:**
  Mean of base (non-shaping) reward components

#### is_shaping()

Checks if this reward includes shaping components.

* **Returns:**
  True if shaping components are present, False otherwise

## gymnasium_jsbsim.assessors

Assessors for computing rewards from state transitions.

This module provides classes that assess states and compute rewards,
including support for sequential and dependent reward components.

### *class* gymnasium_jsbsim.assessors.Assessor

Bases: [`ABC`](https://docs.python.org/3/library/abc.html#abc.ABC)

Interface for Assessors which calculate Rewards from States.

#### *abstractmethod* assess(state: tasks.FlightTask.State, prev_state: tasks.FlightTask.State, is_terminal: [bool](https://docs.python.org/3/library/functions.html#bool)) → [Reward](#gymnasium_jsbsim.rewards.Reward)

Calculates reward from environment’s state, previous state and terminal condition

### *class* gymnasium_jsbsim.assessors.AssessorImpl(base_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)], potential_based_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)] = (), positive_rewards: [bool](https://docs.python.org/3/library/functions.html#bool) = False)

Bases: [`Assessor`](#gymnasium_jsbsim.assessors.Assessor)

Basic implementation of Assessor interface.

Initialised with RewardComponents which allow calculation of the base
(policy-influencing) and shaping rewards (non-policy-influencing) rewards respectively.

#### \_\_init_\_(base_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)], potential_based_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)] = (), positive_rewards: [bool](https://docs.python.org/3/library/functions.html#bool) = False)

Initialises the Assessor with given RewardComponents.
:param base_components: RewardComponents from which Reward is to be calculated
:param potential_based_components: RewardComponents from which a potential-based
reward component is to be calculated from
:param positive_rewards: True if rewards should be in [0.0, 1.0] (0.0 corresponds to
worst behaviour), else rewards will be in [-1.0, 0.0] with 0.0 corresponds to
perfect behaviour. Has no effect one potential difference based components.

#### assess(state: tasks.FlightTask.State, prev_state: tasks.FlightTask.State, is_terminal: [bool](https://docs.python.org/3/library/functions.html#bool)) → [Reward](#gymnasium_jsbsim.rewards.Reward)

Calculates a Reward from the state transition.
:param state: the current State after transition
:param prev_state: the previous State before transition
:param is_terminal: whether the transition to state was terminal
:return: Reward object containing base and potential-based rewards

### *class* gymnasium_jsbsim.assessors.SequentialAssessor(base_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)], potential_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)] = (), base_dependency_map: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), ...]] | [None](https://docs.python.org/3/library/constants.html#None) = None, potential_dependency_map: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), ...]] | [None](https://docs.python.org/3/library/constants.html#None) = None, positive_rewards: [bool](https://docs.python.org/3/library/functions.html#bool) = False)

Bases: [`AssessorImpl`](#gymnasium_jsbsim.assessors.AssessorImpl), [`ABC`](https://docs.python.org/3/library/abc.html#abc.ABC)

Abstract class that allows base and potential components to be assigned
dependencies of other components, such that they are affected by the
other’s values.

Concrete subclasses should implement \_apply_dependents(), which modifies
the ‘normal’ component potentials to account for dependents

#### \_\_init_\_(base_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)], potential_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)] = (), base_dependency_map: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), ...]] | [None](https://docs.python.org/3/library/constants.html#None) = None, potential_dependency_map: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), ...]] | [None](https://docs.python.org/3/library/constants.html#None) = None, positive_rewards: [bool](https://docs.python.org/3/library/functions.html#bool) = False)

Initialises the SequentialAssessor with given RewardComponents and
their dependencies.
:param base_components: RewardComponents from which the non-shaping
part of the Reward is to be calculated
:param potential_components: ErrorComponents from which the shaping
reward is to be calculated, or an empty tuple for no shaping
:param base_dependency_map: maps base components with sequential
dependencies to their dependent components, defaults to
no dependencies
:param potential_dependency_map: maps potential components with sequential
dependencies to their dependent components, defaults to
no dependencies

### *class* gymnasium_jsbsim.assessors.ContinuousSequentialAssessor(base_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)], potential_components: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent)] = (), base_dependency_map: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), ...]] | [None](https://docs.python.org/3/library/constants.html#None) = None, potential_dependency_map: [Dict](https://docs.python.org/3/library/typing.html#typing.Dict)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[RewardComponent](#gymnasium_jsbsim.rewards.RewardComponent), ...]] | [None](https://docs.python.org/3/library/constants.html#None) = None, positive_rewards: [bool](https://docs.python.org/3/library/functions.html#bool) = False)

Bases: [`SequentialAssessor`](#gymnasium_jsbsim.assessors.SequentialAssessor)

A sequential assessor in which shaping components with dependents have their potential
reduced according to their dependent’s potentials through multiplication.

For example a component with a “base” potential of 0.8 and a dependent component at
0.5 have a sequential potential of 0.8 \* 0.5 = 0.4.

## gymnasium_jsbsim.properties

JSBSim property definitions and related classes.

### *class* gymnasium_jsbsim.properties.BoundedProperty(name, description, min, max)

Bases: [`BoundedProperty`](#gymnasium_jsbsim.properties.BoundedProperty)

A property with defined minimum and maximum bounds.

#### get_legal_name()

Get the property name with illegal characters translated.

* **Returns:**
  Property name safe for use as Python attribute

### *class* gymnasium_jsbsim.properties.Property(name, description)

Bases: [`Property`](#gymnasium_jsbsim.properties.Property)

A property without bounds.

#### get_legal_name()

Get the property name with illegal characters translated.

* **Returns:**
  Property name safe for use as Python attribute

### *class* gymnasium_jsbsim.properties.Vector2(x: [float](https://docs.python.org/3/library/functions.html#float), y: [float](https://docs.python.org/3/library/functions.html#float))

Bases: [`object`](https://docs.python.org/3/library/functions.html#object)

2D vector with heading calculation capability.

#### \_\_init_\_(x: [float](https://docs.python.org/3/library/functions.html#float), y: [float](https://docs.python.org/3/library/functions.html#float))

#### heading_deg()

Calculate heading in degrees of vector from origin.

* **Returns:**
  Heading in degrees [0, 360)

#### *static* from_sim(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation)) → [Vector2](#gymnasium_jsbsim.properties.Vector2)

Create a Vector2 from simulation velocity components.

* **Parameters:**
  **sim** – Simulation instance
* **Returns:**
  Vector2 with east and north velocity components

### *class* gymnasium_jsbsim.properties.GeodeticPosition(latitude_deg: [float](https://docs.python.org/3/library/functions.html#float), longitude_deg: [float](https://docs.python.org/3/library/functions.html#float))

Bases: [`object`](https://docs.python.org/3/library/functions.html#object)

Geographic position with latitude and longitude.

#### \_\_init_\_(latitude_deg: [float](https://docs.python.org/3/library/functions.html#float), longitude_deg: [float](https://docs.python.org/3/library/functions.html#float))

#### heading_deg_to(destination: [GeodeticPosition](#gymnasium_jsbsim.properties.GeodeticPosition)) → [float](https://docs.python.org/3/library/functions.html#float)

Determines heading in degrees of course between self and destination.

* **Parameters:**
  **destination** – Target position
* **Returns:**
  Heading in degrees

#### *static* from_sim(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation)) → [GeodeticPosition](#gymnasium_jsbsim.properties.GeodeticPosition)

Return a GeodeticPosition object with lat and lon from simulation.

* **Parameters:**
  **sim** – Simulation instance
* **Returns:**
  GeodeticPosition with current lat/lon

#### \_\_sub_\_(other) → [Vector2](#gymnasium_jsbsim.properties.Vector2)

Returns difference between two coords as (delta_lon, delta_lat).

* **Parameters:**
  **other** – Another GeodeticPosition
* **Returns:**
  Vector2 with longitude and latitude differences

## gymnasium_jsbsim.visualiser

Module for visualisation of JSBSim simulations using FlightGear and Matplotlib.

### *class* gymnasium_jsbsim.visualiser.FlightGearConfig(enable_ai_traffic: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_real_weather: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_random_objects: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_random_vegetation: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_random_buildings: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_panel: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_sound: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_hud_3d: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_clouds: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_clouds_3d: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_horizon_effect: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_enhanced_lighting: [bool](https://docs.python.org/3/library/functions.html#bool) = True, enable_distance_attenuation: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_specular_highlight: [bool](https://docs.python.org/3/library/functions.html#bool) = False, visibility_m: [int](https://docs.python.org/3/library/functions.html#int) = 5000)

Bases: [`object`](https://docs.python.org/3/library/functions.html#object)

Configuration for FlightGear performance and visual settings.

#### enable_ai_traffic *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_real_weather *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_random_objects *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_random_vegetation *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_random_buildings *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_panel *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_sound *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_hud_3d *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_clouds *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_clouds_3d *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_horizon_effect *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_enhanced_lighting *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= True*

#### enable_distance_attenuation *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### enable_specular_highlight *: [bool](https://docs.python.org/3/library/functions.html#bool)* *= False*

#### visibility_m *: [int](https://docs.python.org/3/library/functions.html#int)* *= 5000*

#### *classmethod* performance_mode() → [FlightGearConfig](#gymnasium_jsbsim.visualiser.FlightGearConfig)

Creates a config optimized for performance (all features disabled).

* **Returns:**
  FlightGearConfig with performance-optimized settings

#### *classmethod* quality_mode() → [FlightGearConfig](#gymnasium_jsbsim.visualiser.FlightGearConfig)

Creates a config optimized for visual quality (most features enabled).

* **Returns:**
  FlightGearConfig with quality-optimized settings

#### \_\_init_\_(enable_ai_traffic: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_real_weather: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_random_objects: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_random_vegetation: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_random_buildings: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_panel: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_sound: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_hud_3d: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_clouds: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_clouds_3d: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_horizon_effect: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_enhanced_lighting: [bool](https://docs.python.org/3/library/functions.html#bool) = True, enable_distance_attenuation: [bool](https://docs.python.org/3/library/functions.html#bool) = False, enable_specular_highlight: [bool](https://docs.python.org/3/library/functions.html#bool) = False, visibility_m: [int](https://docs.python.org/3/library/functions.html#int) = 5000) → [None](https://docs.python.org/3/library/constants.html#None)

### *class* gymnasium_jsbsim.visualiser.AxesTuple(axes_state: Axes, axes_stick: Axes, axes_throttle: Axes, axes_rudder: Axes)

Bases: [`NamedTuple`](https://docs.python.org/3/library/typing.html#typing.NamedTuple)

Holds references to figure subplots (axes)

#### axes_state *: Axes*

Alias for field number 0

#### axes_stick *: Axes*

Alias for field number 1

#### axes_throttle *: Axes*

Alias for field number 2

#### axes_rudder *: Axes*

Alias for field number 3

### *class* gymnasium_jsbsim.visualiser.FigureVisualiser(\_: [Simulation](#gymnasium_jsbsim.simulation.Simulation), print_props: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[Property](#gymnasium_jsbsim.properties.Property)])

Bases: [`object`](https://docs.python.org/3/library/functions.html#object)

Class for manging a matplotlib Figure displaying agent state and actions

#### \_\_init_\_(\_: [Simulation](#gymnasium_jsbsim.simulation.Simulation), print_props: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[Property](#gymnasium_jsbsim.properties.Property)])

Initializes FigureVisualiser.

* **Parameters:**
  * **\_** – (unused) Simulation that will be plotted
  * **print_props** – properties which will have their values printed to Figure.
    Must be retrievable from the plotted Simulation object.

#### plot(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation)) → [None](https://docs.python.org/3/library/constants.html#None)

Creates or updates a 3D plot of the episode.

* **Parameters:**
  **sim** – Simulation to plot from

#### close()

Cleans up and closes the figure.

### *class* gymnasium_jsbsim.visualiser.FlightGearVisualiser(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation), print_props: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[Property](#gymnasium_jsbsim.properties.Property)], block_until_loaded: [bool](https://docs.python.org/3/library/functions.html#bool) = True, config: [FlightGearConfig](#gymnasium_jsbsim.visualiser.FlightGearConfig) | [None](https://docs.python.org/3/library/constants.html#None) = None)

Bases: [`object`](https://docs.python.org/3/library/functions.html#object)

Class for visualising aircraft using the FlightGear simulator.

This visualiser launches FlightGear and waits for it to launch.

It then configures the JSBSim simulation to output data to FlightGear and
starts a FigureVisualiser to plot agent actions.

#### \_\_init_\_(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation), print_props: [Tuple](https://docs.python.org/3/library/typing.html#typing.Tuple)[[Property](#gymnasium_jsbsim.properties.Property)], block_until_loaded: [bool](https://docs.python.org/3/library/functions.html#bool) = True, config: [FlightGearConfig](#gymnasium_jsbsim.visualiser.FlightGearConfig) | [None](https://docs.python.org/3/library/constants.html#None) = None)

Launches FlightGear in subprocess and starts figure for plotting actions.

* **Parameters:**
  * **sim** – Simulation that will be visualised
  * **print_props** – collection of Propertys to be printed to Figure
  * **block_until_loaded** – visualiser will block until it detects that
    FlightGear has loaded if True.
  * **config** – FlightGearConfig for performance/quality settings.
    If None, uses default performance-optimized settings.

#### plot(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation)) → [None](https://docs.python.org/3/library/constants.html#None)

Updates a 3D plot of agent actions.
:param sim: Simulation to plot from

#### configure_simulation_output(sim: [Simulation](#gymnasium_jsbsim.simulation.Simulation))

Configure simulation for FlightGear output.

* **Parameters:**
  **sim** – Simulation to configure

#### close()

Closes FlightGear and figure visualiser.

## gymnasium_jsbsim.utils

Utility functions and classes.

### *class* gymnasium_jsbsim.utils.AttributeFormatter

Bases: [`object`](https://docs.python.org/3/library/functions.html#object)

Replaces illegal characters in an attribute name.

Used through its static method translate().

#### ILLEGAL_CHARS *= '\\\\-/.'*

#### TRANSLATE_TO *= '_\_\_\_'*

#### TRANSLATION_TABLE *= {45: 95, 46: 95, 47: 95, 92: 95}*

#### *static* translate(string: [str](https://docs.python.org/3/library/stdtypes.html#str))

Translates illegal attribute characters to underscores.

* **Parameters:**
  **string** – Input string to translate
* **Returns:**
  Translated string

### gymnasium_jsbsim.utils.is_flightgear_installed() → [bool](https://docs.python.org/3/library/functions.html#bool)

Checks whether FlightGear is installed on the system.

* **Returns:**
  True if FlightGear is installed on the system else False.

### gymnasium_jsbsim.utils.get_env_id(task_type, aircraft, shaping, enable_flightgear) → [str](https://docs.python.org/3/library/stdtypes.html#str)

Creates an env ID from the environment’s components.

* **Parameters:**
  * **task_type** – Task type class
  * **aircraft** – Aircraft instance
  * **shaping** – Shaping enum value
  * **enable_flightgear** – bool, whether FlightGear is enabled
* **Returns:**
  Environment ID string

### gymnasium_jsbsim.utils.product(iterable: [Iterable](https://docs.python.org/3/library/typing.html#typing.Iterable))

Multiplies all elements of iterable and returns result.

Adapted from a code snippet provided by Raymond Hettinger on StackOverflow:
[https://stackoverflow.com/questions/595374/whats-the-function-like-sum-but-for-multiplication-product](https://stackoverflow.com/questions/595374/whats-the-function-like-sum-but-for-multiplication-product)

* **Parameters:**
  **iterable** – Iterable of numbers to multiply
* **Returns:**
  Product of all elements

### gymnasium_jsbsim.utils.reduce_reflex_angle_deg(angle: [float](https://docs.python.org/3/library/functions.html#float)) → [float](https://docs.python.org/3/library/functions.html#float)

Given an angle in degrees, normalises in [-179, 180].

Adapted from a solution by James Polk on StackOverflow:
[https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180](https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180)-degrees#

* **Parameters:**
  **angle** – Angle in degrees
* **Returns:**
  Normalized angle in range [-179, 180]

## gymnasium_jsbsim.agents

Base agent classes for reinforcement learning.

### *class* gymnasium_jsbsim.agents.agents.Agent

Bases: [`ABC`](https://docs.python.org/3/library/abc.html#abc.ABC)

Base class for agents.

#### *abstractmethod* act(state) → ndarray

Select an action based on the current state.

#### *abstractmethod* observe(state, action, reward, done)

Observe the result of taking an action.

### *class* gymnasium_jsbsim.agents.agents.RandomAgent(action_space: [Space](https://gymnasium.farama.org/api/spaces/#gymnasium.spaces.Space))

Bases: [`Agent`](#gymnasium_jsbsim.agents.agents.Agent)

An agent that selects random actions.

The Random object making selection is gym.np_random used by the
Space.sample() method. Its seed is set by gym.

#### \_\_init_\_(action_space: [Space](https://gymnasium.farama.org/api/spaces/#gymnasium.spaces.Space))

#### act(state)

Select an action based on the current state.

#### observe(state, action, reward, done)

Observe the result of taking an action.

### *class* gymnasium_jsbsim.agents.agents.ConstantAgent(action_space: [Box](https://gymnasium.farama.org/api/spaces/fundamental/#gymnasium.spaces.Box))

Bases: [`Agent`](#gymnasium_jsbsim.agents.agents.Agent)

An agent that always selects the same action.

#### \_\_init_\_(action_space: [Box](https://gymnasium.farama.org/api/spaces/fundamental/#gymnasium.spaces.Box))

#### act(state)

Select an action based on the current state.

#### observe(state, action, reward, done)

Observe the result of taking an action.
