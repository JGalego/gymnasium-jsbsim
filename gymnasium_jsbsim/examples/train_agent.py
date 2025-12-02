"""
Training example for aircraft control using PPO (Proximal Policy Optimization).

This example shows how to train a reinforcement learning agent with Stable-Baselines3.

Requirements:
    pip install stable-baselines3

Optional TensorBoard visualization:
    To enable TensorBoard logging, set USE_TENSORBOARD = True below.
    Then view training progress with:
        tensorboard --logdir ./training_logs/
    Open http://localhost:6006 in your browser.
"""

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

import gymnasium_jsbsim  # noqa: F401

# Configuration
ENV_ID = "JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0"
TOTAL_TIMESTEPS = 100_000  # Increase for better performance
N_ENVS = 4  # Number of parallel environments
USE_TENSORBOARD = False  # Set to True to enable TensorBoard logging

print(f"Training PPO agent on {ENV_ID}")
print(f"Total timesteps: {TOTAL_TIMESTEPS:,}")
print(f"Using {N_ENVS} parallel environments")

# Configure TensorBoard logging
tensorboard_log = None
if USE_TENSORBOARD:
    tensorboard_log = "./training_logs/"
    print(f"TensorBoard logging enabled: {tensorboard_log}")
    print("View training progress with: tensorboard --logdir ./training_logs/")
    print("Then open http://localhost:6006 in your browser\n")
else:
    print("TensorBoard logging disabled (set USE_TENSORBOARD = True to enable)\n")

# Create vectorized environment for faster training
env = make_vec_env(ENV_ID, n_envs=N_ENVS)

# Initialize PPO agent
model = PPO(
    "MlpPolicy",  # Use Multi-Layer Perceptron policy
    env,
    learning_rate=3e-4,
    n_steps=2048 // N_ENVS,
    batch_size=64,
    verbose=1,
    tensorboard_log=tensorboard_log,
)

# Train the agent
print("Starting training...")
model.learn(total_timesteps=TOTAL_TIMESTEPS, progress_bar=True)

# Save the trained model
model.save("ppo_aircraft_simple")
print("\n✓ Training complete! Model saved as 'ppo_aircraft_simple.zip'")

# Test the trained agent
print("\nTesting trained agent...")
eval_env = gym.make(ENV_ID)
obs, info = eval_env.reset()

episode_reward: float = 0.0
for step in range(300):
    # Use the trained model to predict actions
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = eval_env.step(action)
    episode_reward += float(reward)

    if terminated or truncated:
        print(f"Episode finished after {step + 1} steps")
        print(f"Total reward: {episode_reward:.2f}")
        break

eval_env.close()

print("\n" + "=" * 60)
print("To evaluate the model later, use:")
print('  model = PPO.load("ppo_aircraft_simple")')
if USE_TENSORBOARD:
    print("\nTo view training metrics:")
    print("  tensorboard --logdir ./training_logs/")
print("=" * 60)
