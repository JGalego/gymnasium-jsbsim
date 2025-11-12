"""
A simple example of using the Gymnasium-JSBSim environment with a random agent.
"""

import gymnasium as gym
import gymnasium_jsbsim

env = gym.make('JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0')

env.reset()
step = 0
done = False
while not done:
    action = env.action_space.sample()  # random action
    state, reward, done, info = env.step(action)
    print(f'Step: {step}, State: {state}, Reward: {reward}, Done: {done}\n')
    step += 1

env.close()