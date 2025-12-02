"""
Alternative FlightGear-compatible random agent example with additional features.

This version includes:
- More detailed status updates
- Episode restart capability
- Better error handling
- Performance statistics

QUICK START:
------------
1. Install FlightGear: sudo apt-get install flightgear (Ubuntu/Debian)
2. Run: python gymnasium_jsbsim/examples/random_agent_flightgear.py

FLIGHTGEAR CONTROLS:
-------------------
- V: Cycle camera views
- Mouse drag: Pan camera
- Mouse wheel: Zoom
- ESC: Exit FlightGear menu
- Pause: Pause simulation

See FLIGHTGEAR_SETUP.md for complete documentation.
"""

import time

import gymnasium as gym
import numpy as np

import gymnasium_jsbsim  # noqa: F401

# Configuration
MAX_STEPS = 200
NUM_EPISODES = 3
STEP_DELAY = 0.05  # Delay between steps for better visualization

# Create the FlightGear-enabled environment
print("=" * 60)
print("Gymnasium-JSBSim FlightGear Random Agent Demo")
print("=" * 60)
print("\nEnvironment: JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0")
print(f"Episodes: {NUM_EPISODES}")
print(f"Max steps per episode: {MAX_STEPS}")
print("=" * 60)

env = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0")

# Print action and observation space info
print("\nAction Space:", env.action_space)
print("Observation Space:", env.observation_space)
print()

# Track statistics
total_rewards = []
episode_lengths = []

# Initialize the environment once
print("Initializing environment...")
env.reset()

# Launch FlightGear on first render
print("Launching FlightGear...")
print("(This may take 15-20 seconds on first launch)")
print("TIP: You can cycle camera views in FlightGear using 'V' key")
# Access the unwrapped environment to call render with parameters
env.unwrapped.render(mode="flightgear", flightgear_blocking=True)
print("FlightGear ready!\n")

try:
    for episode in range(NUM_EPISODES):
        print(f"\n{'='*60}")
        print(f"EPISODE {episode + 1}/{NUM_EPISODES}")
        print(f"{'='*60}")

        # Reset environment
        state, info = env.reset()
        episode_reward = 0

        for step in range(MAX_STEPS):
            # Take a random action
            action = env.action_space.sample()

            # Step the environment
            state, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward

            # Update visualization
            env.unwrapped.render(mode="flightgear")

            # Print step information
            done = terminated or truncated
            status = "DONE" if done else "OK"
            print(
                f"  Step {step:3d} | Reward: {reward:7.3f} | "
                f"Total: {episode_reward:8.2f} | Status: {status}"
            )

            # If the episode is done, exit the loop
            if done:
                episode_lengths.append(step + 1)
                total_rewards.append(episode_reward)
                print(f"\n  Episode finished after {step + 1} steps")
                print(f"  Total reward: {episode_reward:.2f}")
                break

            # Small delay for better visualization
            time.sleep(STEP_DELAY)
        else:
            # Episode reached max steps without terminating
            episode_lengths.append(MAX_STEPS)
            total_rewards.append(episode_reward)
            print(f"\n  Episode reached maximum steps ({MAX_STEPS})")
            print(f"  Total reward: {episode_reward:.2f}")

        # Wait between episodes
        if episode < NUM_EPISODES - 1:
            print("\n  Waiting 2 seconds before next episode...")
            time.sleep(2)

except KeyboardInterrupt:
    print("\n\nInterrupted by user")

finally:
    # Print statistics
    print("\n" + "=" * 60)
    print("STATISTICS")
    print("=" * 60)
    if total_rewards:
        print(f"Episodes completed: {len(total_rewards)}")
        print(f"Average reward: {np.mean(total_rewards):.2f}")
        print(f"Best reward: {np.max(total_rewards):.2f}")
        print(f"Worst reward: {np.min(total_rewards):.2f}")
        print(f"Average episode length: {np.mean(episode_lengths):.1f} steps")
    print("=" * 60)

    # Clean shutdown
    print("\nClosing environment and FlightGear...")
    env.close()
    print("Done!\n")
