"""
Base agent classes for reinforcement learning.
"""

from abc import ABC, abstractmethod

import gymnasium as gym
import numpy as np


class Agent(ABC):
    """Base class for agents."""

    @abstractmethod
    def act(self, state) -> np.ndarray:
        """Select an action based on the current state."""

    @abstractmethod
    def observe(self, state, action, reward, done):
        """Observe the result of taking an action."""


class RandomAgent(Agent):
    """
    An agent that selects random actions.

    The Random object making selection is gym.np_random used by the
    Space.sample() method. Its seed is set by gym.
    """

    def __init__(self, action_space: gym.Space):
        self.action_space = action_space

    def act(self, state):
        return self.action_space.sample()

    def observe(self, state, action, reward, done):
        # This agent type does not learn in response to observations
        pass


class ConstantAgent(Agent):
    """An agent that always selects the same action."""

    def __init__(self, action_space: gym.spaces.Box):
        self.constant_action = (action_space.low + action_space.high) / 2

    def act(self, state):
        return self.constant_action

    def observe(self, state, action, reward, done):
        # this agent type does not learn in response to observations
        pass
