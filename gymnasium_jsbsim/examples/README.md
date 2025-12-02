# Gymnasium-JSBSim Examples

This directory contains examples demonstrating how to use the gymnasium-jsbsim environment for reinforcement learning.

## Available Examples

### 1. Random Agent (`random_agent.py`)

A simple example showing basic environment interaction with a random agent.

**Usage:**

```bash
python gymnasium_jsbsim/examples/random_agent.py
```

**What it demonstrates:**
- Creating an environment
- Taking random actions
- Basic environment loop

### 2. Train Agent (`train_agent.py`)

A training example using PPO (Proximal Policy Optimization) to train an aircraft control agent.

**Requirements:**

```bash
pip install stable-baselines3
```

**Usage:**

```bash
python gymnasium_jsbsim/examples/train_agent.py
```

**Configuration:**

Edit the constants at the top of the file to customize training:

```python
ENV_ID = "JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0"
TOTAL_TIMESTEPS = 100_000  # Increase for better performance
N_ENVS = 4  # Number of parallel environments
USE_TENSORBOARD = False  # Set to True to enable TensorBoard logging
```

**What it demonstrates:**
- Training an agent with Stable-Baselines3
- Using vectorized environments for faster training
- Saving and testing a trained model
- Basic performance evaluation
- Optional TensorBoard logging for monitoring training progress

**Expected results:**
- Training completes in 5-15 minutes (depending on hardware)
- Trained agent should show improved performance over random actions
- Model saved as `ppo_aircraft_simple.zip`

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

## Next Steps

After training an agent, you can:

1. **Visualize behavior**: Load the model and render episodes
2. **Fine-tune**: Continue training from a checkpoint
3. **Deploy**: Use the trained model for inference
4. **Experiment**: Try different algorithms (SAC, TD3, A2C)
5. **Benchmark**: Compare performance across different configurations

## Additional Resources

- [Stable-Baselines3 Documentation](https://stable-baselines3.readthedocs.io/)
- [Gymnasium Documentation](https://gymnasium.farama.org/)
- [PPO Paper](https://arxiv.org/abs/1707.06347)
- [JSBSim Documentation](https://jsbsim.sourceforge.net/)

## Contributing

Have an interesting training example or technique? Contributions are welcome! Please see the main [DEVELOPMENT.md](../../DEVELOPMENT.md) for guidelines.
