import gymnasium.envs.registration
import enum
from gymnasium_jsbsim.tasks import Task, HeadingControlTask, TurnHeadingControlTask
from gymnasium_jsbsim.aircraft import Aircraft, cessna172P
from gymnasium_jsbsim import utils

"""
This script registers all combinations of task, aircraft, shaping settings &c.
with Gymnasium so that they can be instantiated with a gym.make(id) command.

The gymnasium_jsbsim.Envs enum stores all registered environments as members with
their env ID string as value. This allows convenient autocompletion and value safety.
"""

for env_id, (task, plane, shaping, enable_flightgear) in utils.get_env_id_kwargs_map().items():
    if enable_flightgear:
        entry_point = 'gymnasium_jsbsim.environment:JsbSimEnv'
    else:
        entry_point = 'gymnasium_jsbsim.environment:NoFGJsbSimEnv'
    kwargs = dict(task_type=task,
                  aircraft=plane,
                  shaping=shaping)
    gymnasium.envs.registration.register(id=env_id,
                                   entry_point=entry_point,
                                   kwargs=kwargs)

# make an Enum storing every Gym-JSBSim environment ID for convenience and value safety
Envs = enum.Enum.__call__('Envs', [(utils.AttributeFormatter.translate(env_id), env_id)
                                   for env_id in utils.get_env_id_kwargs_map().keys()])
