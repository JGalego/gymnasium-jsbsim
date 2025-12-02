"""
Manual testing utilities.
"""

import time
import unittest
from typing import Optional, Type

import gymnasium_jsbsim.properties as prp
from gymnasium_jsbsim import aircraft, tasks
from gymnasium_jsbsim.agents import ConstantAgent, RandomAgent
from gymnasium_jsbsim.environment import JsbSimEnv
from gymnasium_jsbsim.tests.stubs import BasicFlightTask


class TestJsbSimInstance(unittest.TestCase):
    def setUp(self, task_type: Type[tasks.HeadingControlTask] = BasicFlightTask):  # type: ignore[assignment]
        self.env: Optional[JsbSimEnv] = None
        self.env = JsbSimEnv(task_type)
        self.env.reset()

    def tearDown(self):
        self.env.close()  # type: ignore[union-attr]

    def test_long_episode_random_actions(self):
        self.setUp()
        tic = time.time()
        self.env.reset()  # type: ignore[union-attr]
        for i in range(2000):
            self.env.step(action=self.env.action_space.sample())  # type: ignore[union-attr]
            print(f"jsbsim {i / 10} s\n")
        toc = time.time()
        wall_time = toc - tic
        sim_time = self.env.sim.get_sim_time()  # type: ignore[union-attr]
        print(f"Simulated {sim_time} s of flight in {wall_time} s")

    def test_render_episode(self):
        self.setUp()
        render_every = 5
        self.env.reset()  # type: ignore[union-attr]
        for i in range(1000):
            action = self.env.action_space.sample()  # type: ignore[union-attr]
            obs, _, _, _, _ = self.env.step(action=action)  # type: ignore[union-attr]
            if i % render_every == 0:
                self.env.render(mode="human")  # type: ignore[union-attr]

    def test_render_steady_level_flight_random(self):
        """Runs steady level flight task with a random agent."""
        self.setUp(task_type=tasks.HeadingControlTask)
        agent = RandomAgent(self.env.action_space)  # type: ignore[union-attr]
        render_every = 5
        ep_reward: float = 0.0  # type: ignore[assignment]
        done = False
        state = self.env.reset()  # type: ignore[union-attr]
        step_number = 0
        while not done:
            action = agent.act(state)
            state, reward, terminated, truncated, _info = self.env.step(action)  # type: ignore[union-attr]
            done = terminated or truncated
            ep_reward += reward
            if step_number % render_every == 0:
                self.env.render(mode="human")  # type: ignore[union-attr]
            step_number += 1

    def test_run_episode_steady_level_flight_no_render(self):
        self.setUp(task_type=tasks.HeadingControlTask)
        agent = RandomAgent(self.env.action_space)  # type: ignore[union-attr]
        report_every = 20
        EPISODES = 10

        for _ in range(EPISODES):
            ep_reward: float = 0.0  # type: ignore[assignment]
            done = False
            state = self.env.reset()  # type: ignore[union-attr]
            step_number = 0
            while not done:
                action = agent.act(state)
                state, reward, terminated, truncated, _info = self.env.step(action)  # type: ignore[union-attr]
                done = terminated or truncated
                ep_reward += reward
                if step_number % report_every == 0:
                    print(f"time:\t{self.env.sim.get_sim_time()} s")  # type: ignore[union-attr]
                    print(f"last reward:\t{reward}")
                    print(f"episode reward:\t{ep_reward}")
                step_number += 1


class FlightGearRenderTest(unittest.TestCase):
    def setUp(
        self,
        plane: aircraft.Aircraft = aircraft.cessna172P,
        task_type: Type[tasks.HeadingControlTask] = tasks.TurnHeadingControlTask,
    ):
        self.env: Optional[JsbSimEnv] = None
        self.env = JsbSimEnv(aircraft=plane, task_type=task_type)
        self.env.reset()

    def tearDown(self):
        self.env.close()  # type: ignore[union-attr]

    def test_render_steady_level_flight(self):
        self.setUp(plane=aircraft.cessna172P, task_type=tasks.HeadingControlTask)
        agent = ConstantAgent(self.env.action_space)  # type: ignore[union-attr]
        render_every = 5
        report_every = 20
        EPISODES = 999

        for _ in range(EPISODES):
            ep_reward: float = 0.0  # type: ignore[assignment]
            done = False
            state = self.env.reset()  # type: ignore[union-attr]
            self.env.render(mode="flightgear")  # type: ignore[union-attr]
            step_number = 0
            while not done:
                action = agent.act(state)
                state, reward, terminated, truncated, _info = self.env.step(action)  # type: ignore[union-attr]
                done = terminated or truncated
                ep_reward += reward
                if step_number % render_every == 0:
                    self.env.render(mode="flightgear")  # type: ignore[union-attr]
                if step_number % report_every == 0:
                    print(f"time:\t{self.env.sim.get_sim_time()} s")  # type: ignore[union-attr]
                    print(f"last reward:\t{reward}")
                    print(f"episode reward:\t{ep_reward}")
                    print(f"thrust:\t{self.env.sim[prp.engine_thrust_lbs]}")  # type: ignore[union-attr, index]
                    print(f"engine running:\t{self.env.sim[prp.engine_running]}")  # type: ignore[union-attr, index]
                step_number += 1
            print("***\n" f"EPISODE REWARD: {ep_reward}\n" "***")


class TurnHeadingControlTest(unittest.TestCase):
    def setUp(
        self,
        plane: aircraft.Aircraft = aircraft.cessna172P,
        task_type: Type[tasks.HeadingControlTask] = tasks.TurnHeadingControlTask,
        shaping: tasks.Shaping = tasks.Shaping.STANDARD,
    ):
        self.env: Optional[JsbSimEnv] = None
        self.env = JsbSimEnv(aircraft=plane, task_type=task_type, shaping=shaping)
        self.env.reset()

    def tearDown(self):
        self.env.close()  # type: ignore[union-attr]

    def test_render_heading_control(self):
        self.setUp(
            plane=aircraft.a320,
            task_type=tasks.TurnHeadingControlTask,
            shaping=tasks.Shaping.EXTRA_SEQUENTIAL,
        )
        agent = RandomAgent(self.env.action_space)  # type: ignore[union-attr]
        render_every = 5
        report_every = 20
        EPISODES = 50

        for _ in range(EPISODES):
            ep_reward: float = 0.0  # type: ignore[assignment]
            done = False
            state = self.env.reset()  # type: ignore[union-attr]
            self.env.render(mode="flightgear")  # type: ignore[union-attr]
            step_number = 0
            while not done:
                action = agent.act(state)
                state, reward, terminated, truncated, _info = self.env.step(action)  # type: ignore[union-attr]
                done = terminated or truncated
                ep_reward += reward
                if step_number % render_every == 0:
                    self.env.render(mode="flightgear")  # type: ignore[union-attr]
                if step_number % report_every == 0:
                    heading_target = tasks.HeadingControlTask.target_track_deg
                    print(f"time:\t{self.env.sim.get_sim_time()} s")  # type: ignore[union-attr]
                    print(f"last reward:\t{reward}")
                    print(f"episode reward:\t{ep_reward}")
                    print(f"gear status:\t{self.env.sim[prp.gear]}")  # type: ignore[union-attr, index]
                    print(f"thrust eng0:\t{self.env.sim[prp.engine_thrust_lbs]}")  # type: ignore[union-attr, index]
                    print(
                        f'thrust eng1:\t {self.env.sim[prp.Property("propulsion/engine[1]/thrust-lbs", "")]}'  # type: ignore[union-attr, index]
                    )
                    print(f"heading:\t{self.env.sim[prp.heading_deg]}")  # type: ignore[union-attr, index]
                    print(f"target heading:\t{self.env.sim[heading_target]}")  # type: ignore[union-attr, index]
                    print("\n")
                step_number += 1
            print("***\n" f"EPISODE REWARD: {ep_reward}\n" "***\n")
