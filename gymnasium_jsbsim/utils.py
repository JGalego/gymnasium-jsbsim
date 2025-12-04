"""
Utility functions and classes.
"""

import functools
import operator
import shutil
from typing import Iterable


class AttributeFormatter:
    """
    Replaces illegal characters in an attribute name.

    Used through its static method `translate()`.
    """

    ILLEGAL_CHARS = "\\-/."
    TRANSLATE_TO = "_" * len(ILLEGAL_CHARS)
    TRANSLATION_TABLE = str.maketrans(ILLEGAL_CHARS, TRANSLATE_TO)

    @staticmethod
    def translate(string: str):
        """
        Translates illegal attribute characters to underscores.

        :param string: Input string to translate
        :return: Translated string
        """
        return string.translate(AttributeFormatter.TRANSLATION_TABLE)


def is_flightgear_installed() -> bool:
    """
    Checks whether FlightGear is installed on the system.

    :return: `True` if FlightGear is installed on the system else `False`.
    """
    return shutil.which("fgfs") is not None


def get_env_id(task_type, aircraft, shaping, enable_flightgear) -> str:
    """
    Creates an env ID from the environment's components.

    :param task_type: Task type class
    :param aircraft: Aircraft instance
    :param shaping: Shaping enum value
    :param enable_flightgear: bool, whether FlightGear is enabled

    :return: Environment ID string
    """
    if enable_flightgear:
        fg_setting = "FG"
    else:
        fg_setting = "NoFG"
    return f"JSBSim-{task_type.__name__}-{aircraft.name}-{shaping}-{fg_setting}-v0"


def product(iterable: Iterable):
    """
    Multiplies all elements of iterable and returns result.

    Adapted from a code snippet provided by Raymond Hettinger on StackOverflow:
    https://stackoverflow.com/questions/595374/whats-the-function-like-sum-but-for-multiplication-product

    :param iterable: Iterable of numbers to multiply
    :return: Product of all elements
    """
    return functools.reduce(operator.mul, iterable, 1)


def reduce_reflex_angle_deg(angle: float) -> float:
    """
    Given an angle in degrees, normalises in [-179, 180].

    Adapted from a solution by James Polk on StackOverflow:
    https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees#

    :param angle: Angle in degrees
    :return: Normalized angle in range `[-179, 180]`
    """
    new_angle = angle % 360
    if new_angle > 180:
        new_angle -= 360
    return new_angle
