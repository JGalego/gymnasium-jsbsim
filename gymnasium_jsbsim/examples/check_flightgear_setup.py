#!/usr/bin/env python3
"""
FlightGear Setup Diagnostic Tool

This script checks if all dependencies are installed correctly for
FlightGear visualization with Gymnasium-JSBSim.

Run this script to diagnose setup issues before running examples.
"""

import importlib
import subprocess
import sys


def check_python_version():
    """Check Python version."""
    print("Checking Python version...")
    version = sys.version_info
    if version.major >= 3 and version.minor >= 10:
        print(f"  ✓ Python {version.major}.{version.minor}.{version.micro}")
        return True
    else:
        print(
            f"  ✗ Python {version.major}.{version.minor}.{version.micro} (requires 3.10+)"
        )
        return False


def check_python_package(package_name, import_name=None):
    """Check if a Python package is installed."""
    if import_name is None:
        import_name = package_name

    print(f"Checking {package_name}...")
    try:
        module = importlib.import_module(import_name)
        version = getattr(module, "__version__", "unknown version")
        print(f"  ✓ {package_name} ({version})")
        return True
    except ImportError:
        print(f"  ✗ {package_name} not installed")
        print(f"     Install with: pip install {package_name}")
        return False


def check_flightgear():
    """Check if FlightGear is installed."""
    print("Checking FlightGear...")
    try:
        result = subprocess.run(
            ["fgfs", "--version"], capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            # Extract version from output
            version_line = result.stdout.split("\n")[0]
            print(f"  ✓ {version_line}")
            return True
        else:
            print("  ✗ FlightGear installed but returned error")
            return False
    except FileNotFoundError:
        print("  ✗ FlightGear (fgfs) not found in PATH")
        print("     Install instructions:")
        print("       Ubuntu/Debian: sudo apt-get install flightgear")
        print("       macOS: brew install --cask flightgear")
        print("       Windows: Download from https://www.flightgear.org/download/")
        return False
    except subprocess.TimeoutExpired:
        print("  ✗ FlightGear check timed out")
        return False
    except Exception as e:
        print(f"  ✗ Error checking FlightGear: {e}")
        return False


def check_jsbsim_command():
    """Check if JSBSim command-line tool is installed."""
    print("Checking JSBSim command-line tool...")
    try:
        result = subprocess.run(
            ["JSBSim", "--version"], capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            version_line = result.stdout.split("\n")[0]
            print(f"  ✓ {version_line}")
            return True
        else:
            print("  ✗ JSBSim installed but returned error")
            return False
    except FileNotFoundError:
        print("  ⚠ JSBSim command-line tool not found (optional)")
        print("     Python JSBSim library may still work")
        return None  # Not critical
    except subprocess.TimeoutExpired:
        print("  ✗ JSBSim check timed out")
        return False
    except Exception as e:
        print(f"  ⚠ Error checking JSBSim: {e}")
        return None


def check_port_availability():
    """Check if FlightGear port 5550 is available."""
    print("Checking UDP port 5550 (FlightGear communication)...")
    try:
        import socket

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", 5550))
        sock.close()
        print("  ✓ Port 5550 is available")
        return True
    except OSError:
        print("  ⚠ Port 5550 is in use")
        print("     Close any running FlightGear instances")
        return False
    except Exception as e:
        print(f"  ⚠ Could not check port: {e}")
        return None


def test_environment_creation():
    """Test creating a Gymnasium-JSBSim environment."""
    print("Testing environment creation...")
    try:
        import gymnasium as gym

        import gymnasium_jsbsim  # noqa: F401

        # Try to create a NoFG environment (doesn't require FlightGear)
        env = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-NoFG-v0")
        env.reset()
        action = env.action_space.sample()
        env.step(action)
        env.close()

        print("  ✓ Successfully created and tested environment")
        return True
    except Exception as e:
        print(f"  ✗ Error creating environment: {e}")
        return False


def test_flightgear_launch(quick=True):
    """Test launching FlightGear (optional, interactive test)."""
    print("Testing FlightGear launch (this will open a window)...")
    try:
        import time

        import gymnasium as gym

        import gymnasium_jsbsim  # noqa: F401

        # Create FG environment
        env = gym.make("JSBSim-HeadingControlTask-Cessna172P-Shaping.STANDARD-FG-v0")
        env.reset()

        # Launch FlightGear
        print("  → Launching FlightGear (wait ~20 seconds)...")
        env.unwrapped.render(mode="flightgear", flightgear_blocking=True)

        # Check if process started
        if (
            hasattr(env.unwrapped, "flightgear_visualiser")
            and env.unwrapped.flightgear_visualiser
        ):
            fg_vis = env.unwrapped.flightgear_visualiser
            if hasattr(fg_vis, "flightgear_process"):
                fg_process = fg_vis.flightgear_process
                if fg_process.poll() is None:
                    print(f"  ✓ FlightGear launched (PID: {fg_process.pid})")

                    if not quick:
                        # Take a few steps to verify visualization
                        print("  → Taking 5 test steps...")
                        for i in range(5):
                            action = env.action_space.sample()
                            env.step(action)
                            env.unwrapped.render(mode="flightgear")
                            time.sleep(0.5)
                        print("  ✓ Visualization working")

                    env.close()
                    return True
                else:
                    print(f"  ✗ FlightGear exited (code: {fg_process.returncode})")
                    env.close()
                    return False

        print("  ✗ FlightGear visualiser not created")
        env.close()
        return False
    except Exception as e:
        print(f"  ✗ Error launching FlightGear: {e}")
        return False


def main():
    """Run all diagnostic checks."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Check FlightGear setup for Gymnasium-JSBSim"
    )
    parser.add_argument(
        "--test-launch",
        action="store_true",
        help="Also test FlightGear launch (opens window)",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("FlightGear Setup Diagnostic Tool")
    print("Gymnasium-JSBSim")
    print("=" * 60)
    print()

    checks = []

    # Core requirements
    checks.append(("Python version", check_python_version()))
    checks.append(("gymnasium", check_python_package("gymnasium")))
    checks.append(("numpy", check_python_package("numpy")))
    checks.append(("matplotlib", check_python_package("matplotlib")))
    checks.append(("jsbsim", check_python_package("jsbsim")))
    checks.append(
        (
            "gymnasium_jsbsim",
            check_python_package("gymnasium_jsbsim", "gymnasium_jsbsim"),
        )
    )

    # FlightGear requirements
    checks.append(("FlightGear", check_flightgear()))
    checks.append(("JSBSim CLI", check_jsbsim_command()))
    checks.append(("Port 5550", check_port_availability()))

    # Environment test
    checks.append(("Environment creation", test_environment_creation()))

    # Optional FlightGear launch test
    if args.test_launch:
        print()
        checks.append(("FlightGear launch", test_flightgear_launch(quick=True)))

    # Summary
    print()
    print("=" * 60)
    print("SUMMARY")
    print("=" * 60)

    passed = sum(1 for name, result in checks if result is True)
    failed = sum(1 for name, result in checks if result is False)
    warnings = sum(1 for name, result in checks if result is None)

    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print(f"Warnings: {warnings}")
    print()

    if failed == 0:
        print("✓ All critical checks passed!")
        print("  You should be able to run FlightGear examples.")
    else:
        print("✗ Some checks failed.")
        print("  Please install missing dependencies.")
        print()
        print("Failed checks:")
        for name, result in checks:
            if result is False:
                print(f"  - {name}")

    if warnings > 0:
        print()
        print("⚠ Some warnings (non-critical):")
        for name, result in checks:
            if result is None:
                print(f"  - {name}")

    print()
    if not args.test_launch and failed == 0:
        print("To test FlightGear launch (opens window), run:")
        print("  python check_flightgear_setup.py --test-launch")
        print()

    print("For more help, see:")
    print("  - FLIGHTGEAR_SETUP.md")
    print("  - https://github.com/JGalego/gymnasium-jsbsim")
    print("=" * 60)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
