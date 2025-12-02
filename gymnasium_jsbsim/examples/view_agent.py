"""
Visualize a trained agent using FlightGear 3D visualization.

This script loads a previously trained model and demonstrates its behavior
with FlightGear real-time visualization.

SETUP:
------
1. Train a model first (or use an existing one):
   python gymnasium_jsbsim/examples/train_agent.py

2. Install FlightGear:
   sudo apt-get install flightgear  # Ubuntu/Debian

3. Run this script with your trained model:
   python gymnasium_jsbsim/examples/view_agent.py --model ppo_aircraft_simple.zip

REQUIREMENTS:
-------------
- A trained model file (e.g., ppo_aircraft_simple.zip)
- FlightGear installed and accessible
- X11 display available (local machine or X11 forwarding)

For remote servers without display, use:
   xvfb-run -a python gymnasium_jsbsim/examples/view_agent.py

See FLIGHTGEAR_SETUP.md for detailed setup instructions.
"""

import argparse
import os
import time

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO

import gymnasium_jsbsim  # noqa: F401

# Default configuration
DEFAULT_MODEL_PATH = "ppo_aircraft_simple.zip"
DEFAULT_ENV_ID = "JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0"
DEFAULT_NUM_EPISODES = 3
DEFAULT_MAX_STEPS = 300
DEFAULT_STEP_DELAY = 0.05


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Visualize a trained aircraft control agent with FlightGear",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--model",
        type=str,
        default=DEFAULT_MODEL_PATH,
        help="Path to trained model file",
    )
    parser.add_argument(
        "--env", type=str, default=DEFAULT_ENV_ID, help="Environment ID"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=DEFAULT_NUM_EPISODES,
        help="Number of episodes to visualize",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=DEFAULT_MAX_STEPS,
        help="Maximum steps per episode",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=DEFAULT_STEP_DELAY,
        help="Delay between steps in seconds",
    )
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="Use stochastic actions (default: deterministic)",
    )

    return parser.parse_args()


def main():
    """Main visualization function."""
    args = parse_args()

    # Print header
    print("=" * 70)
    print("FLIGHTGEAR VISUALIZATION OF TRAINED AGENT")
    print("=" * 70)
    print()

    # Check if model exists
    if not os.path.exists(args.model):
        print(f"✗ Error: Model file not found: {args.model}")
        print("\nAvailable models in current directory:")
        for f in os.listdir("."):
            if f.endswith(".zip"):
                print(f"  - {f}")
        print("\nTrain a model first with:")
        print("  python gymnasium_jsbsim/examples/train_agent.py")
        return 1

    # Load the trained model
    print(f"Loading model: {args.model}")
    try:
        model = PPO.load(args.model)
        print("✓ Model loaded successfully\n")
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        return 1

    # Create FlightGear-enabled environment
    print(f"Creating environment: {args.env}")
    try:
        env = gym.make(args.env)
        print("✓ Environment created\n")
    except Exception as e:
        print(f"✗ Error creating environment: {e}")
        print("\nMake sure the environment ID is correct.")
        print("Available FG environments:")
        print("  - JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0")
        print("  - JSBSim-HeadingControlTask-F15-Shaping.STANDARD-FG-v0")
        print("  - JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0")
        return 1

    # Initialize environment
    print("Initializing environment...")
    obs, info = env.reset()
    print("✓ Environment initialized\n")

    # Launch FlightGear
    print("Launching FlightGear...")
    print("(This may take 15-20 seconds on first launch)")
    print("TIP: Use 'V' key in FlightGear to cycle through camera views")
    print()

    try:
        env.unwrapped.render(mode="flightgear", flightgear_blocking=True)

        # Verify FlightGear started
        if (
            hasattr(env.unwrapped, "flightgear_visualiser")
            and env.unwrapped.flightgear_visualiser
        ):
            fg_process = env.unwrapped.flightgear_visualiser.flightgear_process
            if fg_process and fg_process.poll() is None:
                print("✓ FlightGear launched successfully!")
            else:
                print("⚠ Warning: FlightGear process may have exited")
        else:
            print("⚠ Warning: FlightGear visualiser not created")

        print("✓ Ready to visualize!\n")

    except Exception as e:
        print(f"✗ Error launching FlightGear: {e}")
        print("\nTroubleshooting:")
        print("  - Check FlightGear is installed: fgfs --version")
        print("  - Ensure X11 display is available: echo $DISPLAY")
        print("  - On remote server, use: xvfb-run -a python ...")
        env.close()
        return 1

    # Run visualization episodes
    episode_rewards = []
    episode_lengths = []

    try:
        for episode in range(args.episodes):
            print(f"\n{'=' * 70}")
            print(f"EPISODE {episode + 1}/{args.episodes}")
            print(f"{'=' * 70}")

            # Reset environment for each episode
            if episode > 0:
                obs, info = env.reset()
                env.unwrapped.render(mode="flightgear")

            episode_reward = 0.0
            step = 0

            while step < args.max_steps:
                # Predict action using trained model
                action, _states = model.predict(obs, deterministic=not args.stochastic)

                # Execute action
                obs, reward, terminated, truncated, info = env.step(action)
                episode_reward += float(reward)

                # Update FlightGear visualization
                env.unwrapped.render(mode="flightgear")

                # Print progress
                done = terminated or truncated
                status = "✓ DONE" if done else "→"
                print(
                    f"  Step {step:3d} | Reward: {reward:7.3f} | "
                    f"Total: {episode_reward:8.2f} | {status}"
                )

                step += 1

                if done:
                    episode_rewards.append(episode_reward)
                    episode_lengths.append(step)
                    print(f"\n  Episode finished after {step} steps")
                    print(f"  Total reward: {episode_reward:.2f}")
                    break

                # Delay for better visualization
                time.sleep(args.delay)
            else:
                # Max steps reached
                episode_rewards.append(episode_reward)
                episode_lengths.append(step)
                print(f"\n  Episode reached maximum steps ({args.max_steps})")
                print(f"  Total reward: {episode_reward:.2f}")

            # Wait between episodes
            if episode < args.episodes - 1:
                print("\n  Waiting 3 seconds before next episode...")
                time.sleep(3)

    except KeyboardInterrupt:
        print("\n\n⚠ Visualization interrupted by user")

    finally:
        # Print summary
        print("\n" + "=" * 70)
        print("SUMMARY")
        print("=" * 70)

        print("\nConfiguration:")
        print(f"  Model: {args.model}")
        print(f"  Environment: {args.env}")
        print(f"  Policy: {'Stochastic' if args.stochastic else 'Deterministic'}")

        if episode_rewards:
            print("\nResults:")
            print(f"  Episodes completed: {len(episode_rewards)}")
            print(f"  Average reward: {np.mean(episode_rewards):.2f}")
            print(f"  Best reward: {np.max(episode_rewards):.2f}")
            print(f"  Worst reward: {np.min(episode_rewards):.2f}")
            print(f"  Average episode length: {np.mean(episode_lengths):.1f} steps")

        print("\nOptions:")
        print("  • Try different environments (--env)")
        print("  • Adjust visualization speed (--delay)")
        print("  • Run more episodes (--episodes)")
        print("  • Use stochastic policy (--stochastic)")

        print("=" * 70)

        # Clean shutdown
        print("\nClosing environment and FlightGear...")
        env.close()
        print("✓ Done!\n")

    return 0


if __name__ == "__main__":
    exit(main())
