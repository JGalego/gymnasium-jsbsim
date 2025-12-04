# Gymnasium-JSBSim Documentation

# Contents:

* [API Reference](https://github.com/JGalego/gymnasium-jsbsim/modules.md)
  * [gymnasium_jsbsim](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim)
  * [gymnasium_jsbsim.environment](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.environment)
  * [gymnasium_jsbsim.tasks](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.tasks)
  * [gymnasium_jsbsim.aircraft](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.aircraft)
  * [gymnasium_jsbsim.simulation](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.simulation)
  * [gymnasium_jsbsim.rewards](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.rewards)
  * [gymnasium_jsbsim.assessors](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.assessors)
  * [gymnasium_jsbsim.properties](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.properties)
  * [gymnasium_jsbsim.visualiser](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.visualiser)
  * [gymnasium_jsbsim.utils](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.utils)
  * [gymnasium_jsbsim.agents](https://github.com/JGalego/gymnasium-jsbsim/modules.md#module-gymnasium_jsbsim.agents.agents)

## Welcome to Gymnasium-JSBSim

Gymnasium-JSBSim provides reinforcement learning environments for flight control
using JSBSim flight dynamics model and the Gymnasium API.

## Features

* Multiple aircraft models (Cessna 172P, A320, F-15)
* Configurable flight tasks (heading control, turn heading control)
* Reward shaping options
* FlightGear visualization support
* Compatible with Gymnasium API

## Quick Start

```python
import gymnasium as gym
from gymnasium_jsbsim import Envs

# Create an environment
env = gym.make(Envs.heading_control_cessna172P_standard_nofg)

# Run a simple episode
observation, info = env.reset()
for _ in range(100):
    action = env.action_space.sample()
    observation, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        break
env.close()
```

## API Reference

See the [API Reference](https://github.com/JGalego/gymnasium-jsbsim/modules.md) section for detailed API documentation.

# Indices and tables

* [Index](https://github.com/JGalego/gymnasium-jsbsim/genindex.md)
* [Module Index](https://github.com/JGalego/gymnasium-jsbsim/py-modindex.md)
* [Search Page](https://github.com/JGalego/gymnasium-jsbsim/search.md)
