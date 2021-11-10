import os
import numpy as np

from models.map import *
from exceptions.map_not_found import *


def locate_map(map_name):
    """
    Get the absolute path of a map file given its full name.
    """
    root_path = os.path.abspath(os.path.dirname(__file__)).split('utils')[0] + 'maps\\'
    file_path = None
    for root, sub, files in os.walk(root_path):
        for file in files:
            name = os.path.basename(root + file)
            if name == map_name:
                file_path = root + file
                break
    if file_path is not None:
        return file_path
    else:
        raise MapNotFound()


def read_asc(file_path, header_len=5):
    """
    Read a .asc file as a Map object.
    """
    file = open(file_path)
    line_index = 0
    values = []
    for line in file:
        if line_index < header_len:
            values.append(line.split()[1])
    n_cols = int(values[0])
    n_rows = int(values[1])
    x_llcorner = int(values[2])
    y_llcorner = int(values[3])
    resolution = int(values[4])
    data = np.loadtxt(file_path, skiprows=header_len)
    file.close()
    return Map(n_cols, n_rows, x_llcorner, y_llcorner, resolution, data)
