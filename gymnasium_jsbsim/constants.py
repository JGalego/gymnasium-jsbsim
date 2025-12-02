"""
Constants used throughout the gymnasium-jsbsim package.

This module centralizes all constant values used across the codebase for
better maintainability and consistency.
"""

# ============================================================================
# Simulation Constants
# ============================================================================
JSBSIM_DT_HZ = 60  # JSBSim integration frequency in Hz

# ============================================================================
# Simulation Class Constants
# ============================================================================
OUTPUT_FILE = "flightgear.xml"
LONGITUDINAL = "longitudinal"
FULL = "full"

# ============================================================================
# Aircraft Constants
# ============================================================================
KTS_TO_M_PER_S = 0.51444
KTS_TO_FT_PER_S = 1.6878

# ============================================================================
# Task Constants
# ============================================================================
# HeadingControlTask
INITIAL_ALTITUDE_FT = 5000
THROTTLE_CMD = 0.8
MIXTURE_CMD = 0.8
INITIAL_HEADING_DEG = 270
DEFAULT_EPISODE_TIME_S = 60.0
ALTITUDE_SCALING_FT = 150
TRACK_ERROR_SCALING_DEG = 8
ROLL_ERROR_SCALING_RAD = 0.15  # approx. 8 deg
SIDESLIP_ERROR_SCALING_DEG = 3.0
MIN_STATE_QUALITY = 0.0  # terminate if state 'quality' is less than this
MAX_ALTITUDE_DEVIATION_FT = 1000  # terminate if altitude error exceeds this

# ============================================================================
# Reward Constants
# ============================================================================
POTENTIAL_BASED_DIFFERENCE_TERMINAL_VALUE = 0.0

# ============================================================================
# Visualizer Constants
# ============================================================================
# FigureVisualiser
PLOT_PAUSE_SECONDS = 0.0001
LABEL_TEXT_KWARGS = {
    "fontsize": 18,
    "horizontalalignment": "right",
    "verticalalignment": "baseline",
}
VALUE_TEXT_KWARGS = {
    "fontsize": 18,
    "horizontalalignment": "left",
    "verticalalignment": "baseline",
}
TEXT_X_POSN_LABEL = 0.8
TEXT_X_POSN_VALUE = 0.9
TEXT_Y_POSN_INITIAL = 1.0
TEXT_Y_INCREMENT = -0.1


# FlightGearVisualiser
FG_TYPE = "socket"
FG_DIRECTION = "in"
FG_RATE = 60
FG_SERVER = ""
FG_PORT = 5550
FG_PROTOCOL = "udp"
FG_LOADED_MESSAGE = "loading cities done"
FG_TIME_FACTOR = 1  # sim speed relative to realtime, higher is faster
FG_TIME = "dusk"

# FlightGear Performance Settings
FG_ENABLE_AI_TRAFFIC = False
FG_ENABLE_REAL_WEATHER = False
FG_ENABLE_RANDOM_OBJECTS = False
FG_ENABLE_RANDOM_VEGETATION = False
FG_ENABLE_RANDOM_BUILDINGS = False
FG_ENABLE_PANEL = False
FG_ENABLE_SOUND = False
FG_ENABLE_HUD_3D = False
FG_ENABLE_CLOUDS = False
FG_ENABLE_CLOUDS_3D = False
FG_ENABLE_HORIZON_EFFECT = False
FG_ENABLE_ENHANCED_LIGHTING = True  # will be removed in future versions
FG_ENABLE_DISTANCE_ATTENUATION = False
FG_ENABLE_SPECULAR_HIGHLIGHT = False
FG_VISIBILITY_M = 5000  # Visibility distance in meters
