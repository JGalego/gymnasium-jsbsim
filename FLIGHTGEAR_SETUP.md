# FlightGear Visualization Setup Guide

This guide explains how to set up and use FlightGear 3D visualization with Gymnasium-JSBSim.

## What is FlightGear?

FlightGear is an open-source flight simulator that provides realistic 3D visualization of aircraft. Gymnasium-JSBSim can connect to FlightGear to display your reinforcement learning agent's flight in real-time.

## Installation

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install flightgear
```

### MacOS

Using Homebrew:

```bash
brew install --cask flightgear
```

Alternatively, download from the [FlightGear website](https://www.flightgear.org/download/).

### Windows

Download and install from the [FlightGear website](https://www.flightgear.org/download/).

### Verify Installation

Confirm FlightGear is installed and accessible from the command line:

```bash
fgfs --version
```

You should see output showing the FlightGear version.

```bash
FlightGear version: 2020.3.16
Revision: none
Build-Id: none
Build-Type: Dev
FG_ROOT=/usr/share/games/flightgear
FG_HOME=/home/jgalego/.fgfs
FG_SCENERY=/home/jgalego/.fgfs/TerraSync:/usr/share/games/flightgear/Scenery
SimGear version: 2020.3.16
OSG version: 3.6.5
PLIB version: 185
```

## Using FlightGear with Gymnasium-JSBSim

### 1. Choose a FlightGear-Enabled Environment

When creating an environment, use an environment ID with `FG` instead of `NoFG`:

```python
# FlightGear-enabled (visualization)
env = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0")

# FlightGear-disabled (no visualization, faster training)
env = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0")
```

### 2. Render the Environment

After resetting the environment, call `render()` to launch FlightGear:

```python
env.reset()

# First render call launches FlightGear
# Use env.unwrapped to access the environment's render method with parameters
env.unwrapped.render(mode="flightgear", flightgear_blocking=True)

# Subsequent calls update the visualization
for step in range(MAX_STEPS):
    action = env.action_space.sample()
    state, reward, terminated, truncated, info = env.step(action)
    env.unwrapped.render(mode="flightgear")  # Update visualization

    if terminated or truncated:
        break

env.close()  # Closes FlightGear
```

### 3. Understanding the Parameters

- **`env.unwrapped`**: Accesses the underlying environment to call `render()` with custom parameters. Gymnasium wrappers don't pass through the `mode` and `flightgear_blocking` parameters, so we need to use `unwrapped` to access the original environment's render method.
- **`mode="flightgear"`**: Launches FlightGear for 3D visualization
- **`flightgear_blocking=True`**: Waits for FlightGear to load (recommended for first call)
- **`flightgear_blocking=False`**: Returns immediately without waiting (faster but may cause issues)

## Example Scripts

### Basic Random Agent with FlightGear

See `random_agent.py` for a complete working example.

```bash
python gymnasium_jsbsim/examples/random_agent.py
```

### Training with Periodic Visualization

You can visualize a trained agent without slowing down training:

```python
import gymnasium as gym
import gymnasium_jsbsim

# Train without visualization (faster)
env_train = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0")
# ... training code ...

# Visualize the trained agent
env_viz = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0")
env_viz.reset()
env_viz.unwrapped.render(mode="flightgear", flightgear_blocking=True)

for step in range(100):
    action = trained_model.predict(state)  # Use your trained agent
    state, reward, terminated, truncated, info = env_viz.step(action)
    env_viz.unwrapped.render(mode="flightgear")

    if terminated or truncated:
        break

env_viz.close()
```

## What You'll See

When FlightGear launches, you'll see:

1. **3D Aircraft View**: The aircraft (Cessna 172P, F15, or A320 depending on your environment)
2. **Cockpit View**: First-person view from the cockpit (use View menu to change)
3. **External View**: Third-person camera following the aircraft
4. **Matplotlib Window**: A separate window showing control surface positions (ailerons, elevator, rudder, throttle)

## Common Issues and Solutions

### FlightGear Won't Launch

**Problem**: FlightGear doesn't start when calling `env.render()`

**Solutions**:
- Verify FlightGear is installed: `fgfs --version`
- Check if FlightGear is already running (close it first)
- Ensure no other process is using UDP port 5550
- Try increasing the blocking timeout in the code

### Port Already in Use

**Problem**: Error about port 5550 already in use

**Solutions**:
- Close any running FlightGear instances
- Check for other processes using port 5550: `lsof -i :5550` (Linux/macOS) or `netstat -ano | findstr 5550` (Windows)
- Wait a few seconds after closing FlightGear before restarting

### FlightGear Launches but Shows No Aircraft Movement

**Problem**: FlightGear window opens but aircraft doesn't move

**Solutions**:
- Ensure you're calling `env.unwrapped.render(mode="flightgear")` after each `env.step()`
- Check that you're using an FG environment (not NoFG)
- Verify JSBSim is properly sending data (check console for errors)

### FlightGear is Slow or Laggy

**Problem**: Visualization is choppy or slow

**Solutions**:
- Reduce graphics settings in FlightGear (View > Rendering Options)
- Disable shadows and other visual effects
- Use a simpler aircraft model (Cessna 172P is lighter than F15 or A320)
- Add a small `time.sleep()` between steps to slow down the simulation

### Training is Too Slow with FlightGear

**Problem**: Training takes too long when visualizing

**Solution**: Use `NoFG` environments for training, `FG` only for visualization:

```python
# Fast training without visualization
train_env = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0")

# Visualize after training
viz_env = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0")
```

## Technical Details

### Communication Protocol

Gymnasium-JSBSim communicates with FlightGear via UDP socket on port 5550 using JSBSim's native FlightGear output protocol. The data is sent at 60 Hz.

### FlightGear Launch Parameters

When you call `env.unwrapped.render(mode="flightgear")`, Gymnasium-JSBSim launches FlightGear with these parameters:

```bash
fgfs \
  --aircraft=c172p \
  --native-fdm=socket,in,60,,5550,udp \
  --fdm=external \
  --disable-ai-traffic \
  --disable-real-weather-fetch \
  --timeofday=noon
```

### Configuration Files

The FlightGear output configuration is defined in `gymnasium_jsbsim/flightgear.xml`.

## Advanced Usage

### Custom FlightGear Settings

To modify FlightGear launch parameters, edit the `_create_cmd_line_args()` method in `gymnasium_jsbsim/visualiser.py`.

### Using Different Camera Views

In FlightGear, use the View menu or keyboard shortcuts:
- `v`: Cycle through views
- `V` (Shift+v): Cycle backwards through views
- `Ctrl+v`: Reset view
- Mouse drag: Pan camera
- Mouse wheel: Zoom in/out

### Recording Flight Data

FlightGear can record your flights. In the FlightGear menu:
1. File > Flight Recorder
2. Configure recording settings
3. Start recording
4. Playback saved flights later

## Performance Tips

1. **Close unnecessary programs**: FlightGear is graphics-intensive
2. **Use simpler aircraft**: Cessna 172P renders faster than F15 or A320
3. **Reduce FlightGear graphics settings**: Lower detail and effects
4. **Add delays**: Use `time.sleep(0.05)` between steps for smoother visualization
5. **Train without FlightGear**: Use NoFG environments for training, FG only for demos

## Further Resources

- [FlightGear Official Website](https://www.flightgear.org/)
- [FlightGear Documentation](https://wiki.flightgear.org/)
- [JSBSim Documentation](https://jsbsim.sourceforge.net/)
- [Gymnasium-JSBSim Repository](https://github.com/JGalego/gymnasium-jsbsim)

## Getting Help

If you encounter issues not covered here:

1. Check the [GitHub Issues](https://github.com/JGalego/gymnasium-jsbsim/issues)
2. Verify JSBSim is installed correctly: `python -c "import jsbsim"`
3. Check FlightGear logs in its output directory
4. Try running FlightGear standalone to verify it works independently
