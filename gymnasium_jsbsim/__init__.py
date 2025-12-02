"""
This module registers all combinations of task, aircraft, shaping settings, etc.
with Gymnasium so that they can be instantiated with a gym.make(id) command.

The gymnasium_jsbsim.Envs enum stores all registered environments as members with
their env ID string as value. This allows convenient autocompletion and value safety.
"""

__version__ = "0.1.0"

import enum
from typing import Dict, Tuple

import gymnasium.envs.registration

from gymnasium_jsbsim import utils
from gymnasium_jsbsim.aircraft import a320, cessna172P, f15
from gymnasium_jsbsim.tasks import HeadingControlTask, Shaping, TurnHeadingControlTask


def get_env_id_kwargs_map() -> Dict[str, Tuple]:
    """
    Returns all environment IDs mapped to tuple of (task, aircraft, shaping, flightgear).
    """
    env_map = {}
    for task_type in (HeadingControlTask, TurnHeadingControlTask):
        for _plane in (cessna172P, a320, f15):
            for _shaping in (Shaping.STANDARD, Shaping.EXTRA, Shaping.EXTRA_SEQUENTIAL):
                for _enable_flightgear in (True, False):
                    _env_id = utils.get_env_id(
                        task_type, _plane, _shaping, _enable_flightgear
                    )
                    assert _env_id not in env_map
                    env_map[_env_id] = (task_type, _plane, _shaping, _enable_flightgear)
    return env_map


for env_id, (
    task,
    plane,
    shaping,
    enable_flightgear,
) in get_env_id_kwargs_map().items():
    if enable_flightgear:
        ENTRY_POINT = "gymnasium_jsbsim.environment:JsbSimEnv"
    else:
        ENTRY_POINT = "gymnasium_jsbsim.environment:NoFGJsbSimEnv"
    kwargs = {"task_type": task, "aircraft": plane, "shaping": shaping}
    gymnasium.envs.registration.register(
        id=env_id, entry_point=ENTRY_POINT, kwargs=kwargs
    )

# Make an Enum storing every Gym-JSBSim environment ID for convenience and value safety
Envs = enum.Enum(
    "Envs",
    [
        (utils.AttributeFormatter.translate(env_id), env_id)
        for env_id in get_env_id_kwargs_map()
    ],
)
