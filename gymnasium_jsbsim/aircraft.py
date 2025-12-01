"""
Aircraft module defining aircraft configurations for JSBSim simulation.
"""
import collections
from gymnasium_jsbsim import constants


class Aircraft(collections.namedtuple('Aircraft', ['jsbsim_id', 'flightgear_id',
                                                   'name', 'cruise_speed_kts'])):
    """
    Aircraft configuration containing identifiers and performance characteristics.
    """

    def get_max_distance_m(self, episode_time_s: float) -> float:
        """
        Estimates the maximum distance this aircraft can travel in an episode.
        
        Args:
            episode_time_s: Episode duration in seconds
            
        Returns:
            Maximum distance in meters
        """
        margin = 0.1
        return self.cruise_speed_kts * constants.KTS_TO_M_PER_S * episode_time_s * (1 + margin)

    def get_cruise_speed_fps(self) -> float:
        """
        Get the cruise speed in feet per second.
        
        Returns:
            Cruise speed in feet per second
        """
        return self.cruise_speed_kts * constants.KTS_TO_FT_PER_S


cessna172P = Aircraft('c172p', 'c172p', 'Cessna172P', 120)
f15 = Aircraft('f15', 'f15c', 'F15', 220)   # observed speed at engine settings used alt = 5k ft
a320 = Aircraft('A320', 'A320-211', 'A320', 490)
