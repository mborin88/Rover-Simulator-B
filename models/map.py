from exceptions.non_integer_indexes import *
from exceptions.indexes_out_of_range import *


class Map:
    """
    A map class.
    """
    def __init__(self, n_cols, n_rows, x_llcorner, y_llcorner, resolution, data):
        self._n_cols = n_cols                            # Columns of grid map.
        self._n_rows = n_rows                            # Rows of grid map.
        self._x_llcorner = x_llcorner                    # Coordinates of lower left corner.
        self._y_llcorner = y_llcorner
        self._resolution = resolution                    # Resolution, in meter.
        self._x_range = self._n_cols * self._resolution  # Range of map. Notice: top & right-hand side boundaries
        self._y_range = self._n_rows * self._resolution  # are not included.
        self._data = data                                # Geo-spatial data for each grid point.

    def __str__(self):
        return 'Map Specifications:' + '\n' + \
               'Lower left easting:  ' + str(self._x_llcorner) + ' (m)' + '\n' + \
               'Lower left northing: ' + str(self._y_llcorner) + ' (m)' + '\n' + \
               'Easting range: ' + str(self._x_range) + ' (m)' + '\n' + \
               'Northing range: ' + str(self._y_range) + ' (m)' + '\n' + \
               'Resolution: ' + str(self._resolution) + ' (m)' + '.'

    @property
    def n_cols(self):
        return self._n_cols

    @property
    def n_rows(self):
        return self._n_rows

    @property
    def x_llcorner(self):
        return self._x_llcorner

    @property
    def y_llcorner(self):
        return self._y_llcorner

    @property
    def resolution(self):
        return self._resolution

    @property
    def x_range(self):
        return self._x_range

    @property
    def y_range(self):
        return self._y_range

    @property
    def data(self):
        return self._data

    def is_valid_index(self, row, col):
        """
        See if an index-pair is valid.
        """
        if isinstance(row, int) and isinstance(col, int):
            if (0 <= row <= self._n_rows - 1) and (0 <= col <= self._n_cols - 1):
                return True
            else:
                return -2
        else:
            return -1

    def coordinate2index(self, easting, northing):
        """
        Change plane coordinates into array indexes.
        """
        e, n = int((easting - self._x_llcorner) / self._resolution), \
            int((northing - self._y_llcorner) / self._resolution)
        row, col = self._n_rows - n - 1, e - 1
        if self.is_valid_index(row, col):
            return [row, col]
        elif self.is_valid_index(row, col) == -1:
            raise NonIntegerIndexes()
        elif self.is_valid_index(row, col) == -2:
            raise IndexesOutOfRange()

    def index2coordinate(self, row, col):
        """
        Change array indexes into plane coordinates.
        These are the coordinates at the lower left corner of each cell.
        """
        if self.is_valid_index(row, col):
            e, n = col, self._n_rows - row
            easting, northing = e * self._resolution + self._x_llcorner, \
                n * self._resolution + self._y_llcorner
            return [easting, northing]
        elif self.is_valid_index(row, col) == -1:
            raise NonIntegerIndexes()
        elif self.is_valid_index(row, col) == -2:
            raise IndexesOutOfRange()

    def get_data(self, easting, northing):
        """
        Get data at given coordinates.
        """
        row, col = self.coordinate2index(easting, northing)
        if self.is_valid_index(row, col):
            return self._data[row, col]
        elif self.is_valid_index(row, col) == -1:
            raise NonIntegerIndexes()
        elif self.is_valid_index(row, col) == -2:
            raise IndexesOutOfRange()
