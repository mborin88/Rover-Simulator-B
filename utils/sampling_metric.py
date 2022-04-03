#Generation of sampling metric
#Could repreent greenhouse gasses, temperature, moisture...
#options were to keep between -1:1:.01 and then scale the position to between here
import numpy as np
import sys
import os
import matplotlib.pyplot as plt
import scipy.stats as stats
sys.path.append(str(os.getcwd()))
from load_map import *

class  Sampling_Metric():
    """
    A sampling metric class
    Where 
    """
    def __init__(self, mu, cov, x_min, x_max, y_min, y_max):
        self._distribution = stats.multivariate_normal(mu, cov)
        self._x_range, self._y_range = np.mgrid[x_min:x_max:1, y_min:y_max:1]
        self._multiplier = 1e8

    @property
    def distribution(self):
        return self._distribution

    def sample(self, x, y):
        return self._multiplier * self.distribution.pdf([x, y])


    def visualise(self):
        pos = np.dstack((self._x_range, self._y_range))
        fig = plt.figure()
        ax = fig.add_subplot(111)
        contf = ax.contourf(self._x_range, self._y_range, self._multiplier * self.distribution.pdf(pos))
        plt.colorbar(contf, label='Measurement')
        ax.set_xlabel('Easting (m)')
        ax.set_ylabel('Northing (m)')
        plt.show()

if __name__ == '__main__':
    #print("Main")
    area = 'TL16NE'
    map_terrain = read_asc(locate_map(area + '_elevation' + '.asc'))
    map_landcover = read_asc(locate_map(area + '_landcover' + '.asc'))
    x_min, x_max = map_terrain.x_llcorner, map_terrain.x_llcorner + map_terrain.x_range
    y_min, y_max = map_terrain.y_llcorner, map_terrain.y_llcorner + map_terrain.y_range

    range = x_max - x_min
    mean = [int((x_min+x_max)/2), int((y_min+y_max)/2)]
    covariance = [[1, 0], [0, 1]]
    covariance = list(np.array(covariance) * range * 1000)

    distribution = Sampling_Metric(mean, covariance, x_min, x_max, y_min, y_max)
    print(distribution.sample(mean[0], mean[1]))
    distribution.visualise()
    


