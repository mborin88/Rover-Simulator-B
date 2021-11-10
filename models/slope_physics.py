from math import *


GRAVITY = 9.8  # m/s^2


class SlopePhysics:
    """
    A slope physics class.
    """
    def __init__(self, world):
        self._physical_world = world                       # A world object.
        self._terrain = world.terrain                      # The terrain info, a map object.
        self._resolution = self._terrain.resolution        # The resolution of map.
        self._x_min = self._terrain.x_llcorner             # Lower left easting.
        self._y_min = self._terrain.y_llcorner             # Lower left northing.
        self._x_max = self._x_min + self._terrain.x_range  # Upper right easting.
        self._y_max = self._y_min + self._terrain.y_range  # Upper right northing.

    def easting_slope(self, easting, northing):
        """
        Calculate slope in easting direction.
        """
        resolution = self._resolution
        if easting >= (self._x_max - resolution):
            return 0
        else:
            e_slope = (self._terrain.get_data(easting + resolution, northing)
                       - self._terrain.get_data(easting, northing)) / resolution
            return e_slope

    def northing_slope(self, easting, northing):
        """
        Calculate slope in northing direction.
        """
        resolution = self._resolution
        if northing >= (self._y_max - resolution):
            return 0
        else:
            n_slope = (self._terrain.get_data(easting, northing + resolution)
                       - self._terrain.get_data(easting, northing)) / resolution
        return n_slope

    def generate_acceleration(self, slope):
        """
        Generate acceleration.
        """
        angle = atan(slope)
        return -GRAVITY * sin(angle)

    def generate_friction(self, slope, mu=0.1):
        """
        Generate friction.
        """
        angle = atan(slope)
        return -mu * GRAVITY * abs(cos(angle))
