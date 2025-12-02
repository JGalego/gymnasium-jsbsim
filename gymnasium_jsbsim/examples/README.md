# Gymnasium-JSBSim Examples

This directory contains examples demonstrating how to use the gymnasium-jsbsim environment for reinforcement learning.

## Quick Setup Check

Before running examples with FlightGear, verify your setup:

```bash
# Basic checks (fast)
python gymnasium_jsbsim/examples/check_flightgear_setup.py

# Also test FlightGear launch (opens window)
python gymnasium_jsbsim/examples/check_flightgear_setup.py --test-launch
```

This diagnostic tool checks:
- Python version and packages
- FlightGear installation
- JSBSim installation
- Network port availability
- Environment creation
- FlightGear launch test (with `--test-launch`)

See [FLIGHTGEAR_SETUP.md](FLIGHTGEAR_SETUP.md) for detailed setup instructions.

## Available Examples

### 1. Random Agent (`random_agent.py`)

A simple example showing basic environment interaction with a random agent (no visualization).

**Usage:**

```bash
python gymnasium_jsbsim/examples/random_agent.py
```

**What it demonstrates:**
- Creating a basic environment
- Taking random actions
- Basic environment loop
- Step-by-step observation and reward printing

### 2. Random Agent with FlightGear (`random_agent_flightgear.py`)

Random agent with FlightGear 3D visualization and performance tracking.

**Requirements:**

```bash
# Install FlightGear (see FLIGHTGEAR_SETUP.md for detailed instructions)
# Ubuntu/Debian:
sudo apt-get install flightgear

# macOS:
brew install --cask flightgear

# Verify installation:
fgfs --version
```

**Usage:**

```bash
python gymnasium_jsbsim/examples/random_agent_flightgear.py
```

**What it demonstrates:**
- FlightGear 3D visualization
- Running multiple episodes
- Performance statistics collection
- Proper error handling and cleanup
- Control surface visualization

**See also:** [FLIGHTGEAR_SETUP.md](FLIGHTGEAR_SETUP.md) for complete FlightGear setup and troubleshooting

### 3. Train Agent (`train_agent.py`)

Train an aircraft control agent using PPO (Proximal Policy Optimization).

**Requirements:**

```bash
pip install stable-baselines3
```

**Usage:**

```bash
python gymnasium_jsbsim/examples/train_agent.py
```

**What it demonstrates:**
- Training with Stable-Baselines3
- Vectorized environments for faster training
- Model saving and evaluation
- Optional TensorBoard logging

**Configuration:**

```python
ENV_ID = "JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0"
TOTAL_TIMESTEPS = 100_000  # Increase for better performance
N_ENVS = 4  # Number of parallel environments
USE_TENSORBOARD = False  # Enable TensorBoard logging
```

**Expected output:**
- Training time: 5-15 minutes (hardware dependent)
- Model saved as `ppo_aircraft_simple.zip`
- Final test episode results

### 4. View Trained Agent (`view_agent.py`)

Visualize any trained model with FlightGear 3D visualization.

**Requirements:**

```bash
pip install stable-baselines3
sudo apt-get install flightgear  # Ubuntu/Debian
```

**Usage:**

```bash
# Basic usage (visualize default model)
python gymnasium_jsbsim/examples/view_agent.py

# Specify model and options
python gymnasium_jsbsim/examples/view_agent.py \
    --model ppo_aircraft_simple.zip \
    --env JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0 \
    --episodes 5 \
    --delay 0.05
```

**What it demonstrates:**
- Loading pre-trained models
- FlightGear 3D visualization
- Performance statistics
- Configurable visualization parameters

**Command-line options:**
- `--model PATH`: Trained model file (default: `ppo_aircraft_simple.zip`)
- `--env ENV_ID`: FlightGear environment to use
- `--episodes N`: Number of episodes (default: 3)
- `--max-steps N`: Max steps per episode (default: 300)
- `--delay SECS`: Delay between steps (default: 0.05)
- `--stochastic`: Use stochastic policy (default: deterministic)

## Monitoring Training Progress

The training script supports optional TensorBoard logging. Enable it by setting `USE_TENSORBOARD = True` in the script.

**View training progress:**
```bash
tensorboard --logdir ./training_logs/
```

Then open http://localhost:6006 in your browser to see:
- Episode reward over time
- Episode length
- Policy loss and entropy
- Value function estimates

## Training Tips

### Improving Performance

1. **Increase training timesteps**: More training generally leads to better performance
   - Edit `TOTAL_TIMESTEPS` in the script (e.g., 500,000 or 1,000,000)

2. **Use more parallel environments**: Faster training with more diverse experience
   - Edit `N_ENVS` in the script (e.g., 8 or 16)

3. **Try different environments**: Each task and aircraft has different difficulty
   - Edit `ENV_ID` in the script, for example:
     - `JSBSim-TurnHeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0`
     - `JSBSim-HeadingControlTask-F15-Shaping.STANDARD-NoFG-v0`

4. **Experiment with reward shaping**: EXTRA or EXTRA_SEQUENTIAL may help learning
   - Use environments with different shaping:
     - `.STANDARD-NoFG-v0` (basic rewards)
     - `.EXTRA-NoFG-v0` (additional potential-based shaping)
     - `.EXTRA_SEQUENTIAL-NoFG-v0` (sequential task rewards)

5. **Enable TensorBoard**: Monitor training progress in real-time
   - Set `USE_TENSORBOARD = True` in the script

### Common Issues

**Training is slow:**
- Use more parallel environments (increase `N_ENVS`)
- Use a machine with more CPU cores
- Disable TensorBoard logging if not needed

**Agent doesn't improve:**
- Increase training timesteps (increase `TOTAL_TIMESTEPS`)
- Try different reward shaping (EXTRA, EXTRA_SEQUENTIAL)
- Adjust PPO hyperparameters in the code
- Try a different environment or aircraft

**Out of memory:**
- Reduce number of parallel environments (decrease `N_ENVS`)
- Reduce `batch_size` in PPO configuration
- Reduce `n_steps` in PPO configuration

## Workflow Summary

**Complete workflow from scratch:**

```bash
# 1. Check setup
python gymnasium_jsbsim/examples/check_flightgear_setup.py

# 2. Try random agent (optional)
python gymnasium_jsbsim/examples/random_agent.py

# 3. Try with FlightGear visualization (optional)
python gymnasium_jsbsim/examples/random_agent_flightgear.py

# 4. Train an agent
python gymnasium_jsbsim/examples/train_agent.py

# 5. Visualize the trained agent with FlightGear
python gymnasium_jsbsim/examples/view_agent.py --model ppo_aircraft_simple.zip
```

## Next Steps

After training an agent:

1. **Visualize**: Use `view_agent.py` to see your trained agent in FlightGear
2. **Fine-tune**: Continue training with `model.learn(total_timesteps=...)`
3. **Experiment**: Try different tasks, aircraft, and reward shaping
4. **Compare**: Train multiple models and benchmark performance
5. **Deploy**: Use the trained model in your applications

## Additional Resources

- [Stable-Baselines3 Documentation](https://stable-baselines3.readthedocs.io/)
- [Gymnasium Documentation](https://gymnasium.farama.org/)
- [PPO Paper](https://arxiv.org/abs/1707.06347)
- [JSBSim Documentation](https://jsbsim.sourceforge.net/)

## Contributing

Have an interesting training example or technique? Contributions are welcome! Please see the main [DEVELOPMENT.md](../../DEVELOPMENT.md) for guidelines.
