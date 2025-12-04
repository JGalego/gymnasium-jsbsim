"""
This module registers all combinations of task, aircraft, shaping settings, etc.
with Gymnasium so that they can be instantiated with a gym.make(id) command.

The gymnasium_jsbsim.Envs enum stores all registered environments as members with
their env ID string as value. This allows convenient autocompletion and value safety.
"""

__version__ = "0.1.1"

import enum
import itertools
from typing import Dict, Tuple

import gymnasium.envs.registration

from gymnasium_jsbsim import utils
from gymnasium_jsbsim.aircraft import a320, cessna172P, f15
from gymnasium_jsbsim.tasks import HeadingControlTask, Shaping, TurnHeadingControlTask


def get_env_id_kwargs_map() -> Dict[str, Tuple]:
    """
    Returns all environment IDs mapped to tuple of (task, aircraft, shaping, flightgear).
    """
    task_types = (HeadingControlTask, TurnHeadingControlTask)
    planes = (cessna172P, a320, f15)
    shapings = (Shaping.STANDARD, Shaping.EXTRA, Shaping.EXTRA_SEQUENTIAL)
    flightgear_settings = (True, False)

    env_map = {}
    for task_type, plane, shaping, enable_flightgear in itertools.product(
        task_types, planes, shapings, flightgear_settings
    ):
        env_id = utils.get_env_id(task_type, plane, shaping, enable_flightgear)
        assert env_id not in env_map
        env_map[env_id] = (task_type, plane, shaping, enable_flightgear)

    return env_map


# Register all environments with Gymnasium
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

# Create an enum of all registered environments for convenient access
Envs = enum.Enum(  # type: ignore[misc]
    "Envs",
    [
        (utils.AttributeFormatter.translate(env_id), env_id)
        for env_id in get_env_id_kwargs_map()
    ],
)
