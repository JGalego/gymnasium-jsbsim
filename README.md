# Gymnasium-JSBSim

Gymnasium-JSBSim provides reinforcement learning environments for the control of fixed-wing aircraft using the JSBSim flight dynamics model.

The package's environments implement the Gymnasium interface allowing environments to be created and interacted with in the usual way:

```python
"""
A simple example of using the Gymnasium-JSBSim environment with a random agent.
"""

import gymnasium as gym

import gymnasium_jsbsim  # pylint: disable=unused-import  # noqa: F401

# Define the maximum number of steps to run
MAX_STEPS = 100

# Create the environment
env = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0")

# Reset the environment to start
env.reset()

for step in range(MAX_STEPS):
    # Take a random action
    action = env.action_space.sample()

    # Step the environment
    state, reward, terminated, truncated, info = env.step(action)

    # Print the results of the step
    done = terminated or truncated
    print(f"Step: {step}, State: {state}, Reward: {reward}, Done: {done}\n")

    # If the episode is done, exit the loop
    if done:
        break

# Close the environment
env.close()
```

Gymnasium-JSBSim optionally provides 3D visualisation of controlled aircraft using the FlightGear simulator.

## Dependencies

* [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics model, including the C++ and Python libraries
* gymnasium, numpy, matplotlib
* (Optional) FlightGear simulator (optional for visualisation)

## Installation

Follow the instructions on the [JSBSim](https://github.com/JSBSim-Team/jsbsim) repository to install JSBSim and its libraries.

Confirm that JSBSim is installed from the terminal:

```
$ JSBSim --version
JSBSim Version: 1.2.3 [GitHub build 1561/commit 570e8115a102df8f877b11e0e59b964ea483e3c0] Jun  7 2025 19:20:54
```

and confirm that its Python library is correctly installed from a Python interpreter or IDE:

```
python -c "import jsbsim"
```

Gymnasium-JSBSim is `pip`-installable using this repository:

```
pip install git+https://github.com/JGalego/gymnasium-jsbsim
```

## Environments

Gymnasium-JSBSim implements two tasks for controlling the altitude and heading of aircraft:

* **Heading Control**: aircraft must fly in a straight line, maintaining its initial altitude and direction of travel (heading)
* **Turn Heading Control**: aircraft must turn to face a random target heading while maintaining their initial altitude

The environment can be configured to use one of three aircraft:

* **Cessna172P** light aircraft
* **F15** fighter jet
* **A320** airliner

Environment ID strings are constructed as follows `JSBSim-{task}-{aircraft}-SHAPING_STANDARD-NoFG-v0`.

For example, to fly a Cessna on the Turn Heading Control task,

```
env = gym.make('JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0')
```

## Visualisation

### 2D

A basic plot of agent actions and current state information can be using `human` render mode by calling `env.render(mode='human')`.

### 3D

3D visualisation requires installation of the FlightGear simulator.

Confirm it is runnable from terminal with:

```
fgfs --version
```

Visualising with FlightGear requires the Gymnasium to be created with a FlightGear-enabled environment ID by changing 'NoFG' -> 'FG'. For example:

```
env = gym.make('JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0')
```

The first call to `env.render(mode='flightgear')` will launch FlightGear and begin visualisation.

## State and Action Space

Gymnasium-JSBSim's environments have a continuous state and action space. The state is a 17-tuple:

```
(name='position/h-sl-ft', description='altitude above mean sea level [ft]', min=-1400, max=85000)
(name='attitude/pitch-rad', description='pitch [rad]', min=-1.5707963267948966, max=1.5707963267948966)
(name='attitude/roll-rad', description='roll [rad]', min=-3.141592653589793, max=3.141592653589793)
(name='velocities/u-fps', description='body frame x-axis velocity [ft/s]', min=-2200, max=2200)
(name='velocities/v-fps', description='body frame y-axis velocity [ft/s]', min=-2200, max=2200)
(name='velocities/w-fps', description='body frame z-axis velocity [ft/s]', min=-2200, max=2200)
(name='velocities/p-rad_sec', description='roll rate [rad/s]', min=-6.283185307179586, max=6.283185307179586)
(name='velocities/q-rad_sec', description='pitch rate [rad/s]', min=-6.283185307179586, max=6.283185307179586)
(name='velocities/r-rad_sec', description='yaw rate [rad/s]', min=-6.283185307179586, max=6.283185307179586)
(name='fcs/left-aileron-pos-norm', description='left aileron position, normalised', min=-1, max=1)
(name='fcs/right-aileron-pos-norm', description='right aileron position, normalised', min=-1, max=1)
(name='fcs/elevator-pos-norm', description='elevator position, normalised', min=-1, max=1)
(name='fcs/rudder-pos-norm', description='rudder position, normalised', min=-1, max=1)
(name='error/altitude-error-ft', description='error to desired altitude [ft]', min=-1400, max=85000)
(name='aero/beta-deg', description='sideslip [deg]', min=-180, max=180)
(name='error/track-error-deg', description='error to desired track [deg]', min=-180, max=180)
(name='info/steps_left', description='steps remaining in episode', min=0, max=300)
```

Actions are 3-tuples of floats in the range [-1,+1] describing commands to move the aircraft's control surfaces (ailerons, elevator, rudder):
```
(name='fcs/aileron-cmd-norm', description='aileron commanded position, normalised', min=-1.0, max=1.0)
(name='fcs/elevator-cmd-norm', description='elevator commanded position, normalised', min=-1.0, max=1.0)
(name='fcs/rudder-cmd-norm', description='rudder commanded position, normalised', min=-1.0, max=1.0)
```

## Other Materials

* Gymnasium-JBSim is a fork of [Gor-Ren's Gym-JSBSim](https://github.com/Gor-Ren/gym-jsbsim).
* Gym-JSBSim was created as part of a MSc dissertation, which can be accessed [here](https://researchportal.bath.ac.uk/en/publications/autonomous-control-of-simulated-fixed-wing-aircraft-using-deep-re).
* A video montage of trained agent behaviour is available [here](https://drive.google.com/open?id=1wEq4Fg31Nf_6jb6bLLO24gt15GaZ-wbv).
