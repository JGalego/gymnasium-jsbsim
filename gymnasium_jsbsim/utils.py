"""
Utility functions and classes for gymnasium-jsbsim.
"""

import functools
import operator
import shutil

from typing import Iterable


class AttributeFormatter:  # pylint: disable=too-few-public-methods
    """
    Replaces characters that would be illegal in an attribute name.

    Used through its static method, translate()
    """
    ILLEGAL_CHARS = '\\-/.'
    TRANSLATE_TO = '_' * len(ILLEGAL_CHARS)
    TRANSLATION_TABLE = str.maketrans(ILLEGAL_CHARS, TRANSLATE_TO)

    @staticmethod
    def translate(string: str):
        """
        Translate illegal attribute characters to underscores.
        
        Args:
            string: Input string to translate
            
        Returns:
            Translated string
        """
        return string.translate(AttributeFormatter.TRANSLATION_TABLE)


def is_flightgear_installed() -> bool:
    """
    Returns True if FlightGear is installed on the system else False.
    """
    return shutil.which('fgfs') is not None


def get_env_id(task_type, aircraft, shaping, enable_flightgear) -> str:
    """
    Creates an env ID from the environment's components.

    Args:
        task_type: Task class, the environment's task
        aircraft: Aircraft namedtuple, the aircraft to be flown
        shaping: HeadingControlTask.Shaping enum, the reward shaping setting
        enable_flightgear: True if FlightGear simulator is enabled for
            visualisation else False
            
    Returns:
        Environment ID string
    """
    if enable_flightgear:
        fg_setting = 'FG'
    else:
        fg_setting = 'NoFG'
    return f'JSBSim-{task_type.__name__}-{aircraft.name}-{shaping}-{fg_setting}-v0'


def product(iterable: Iterable):
    """
    Multiplies all elements of iterable and returns result.

    ATTRIBUTION: code provided by Raymond Hettinger on SO
    https://stackoverflow.com/questions/595374/whats-the-function-like-sum-but-for-multiplication-product
    
    Args:
        iterable: Iterable of numbers to multiply
        
    Returns:
        Product of all elements
    """
    return functools.reduce(operator.mul, iterable, 1)


def reduce_reflex_angle_deg(angle: float) -> float:
    """
    Given an angle in degrees, normalises in [-179, 180].
    
    ATTRIBUTION: solution from James Polk on SO,
    https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees#
    
    Args:
        angle: Angle in degrees
        
    Returns:
        Normalized angle in range [-179, 180]
    """
    new_angle = angle % 360
    if new_angle > 180:
        new_angle -= 360
    return new_angle
