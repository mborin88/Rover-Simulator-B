#Generation of sampling metric
#Could repreent greenhouse gasses, temperature, moisture...
#options were to keep between -1:1:.01 and then scale the position to between here
import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from  scipy.stats import (multivariate_normal as mvn,norm)
from  scipy.stats._multivariate import _squeeze_output
sys.path.append(str(os.getcwd()))
from load_map import *

class Multivariate_skewnorm:
    """
    Additional generation of metric if the gaussian distirbution was skewed.
    Based off: https://gregorygundersen.com/blog/2020/12/29/multivariate-skew-normal/
    Not working yet.
    """
    def __init__(self, shape, x_min, x_max, y_min, y_max, mean, cov=None):
        self.dim   = len(shape)
        self.shape = np.asarray(shape)
        self._multiplier = 1e7
        self.mean  = mean
        self._x_range, self._y_range = np.mgrid[x_min:x_max:1, y_min:y_max:1]
        self.cov   = np.eye(self.dim) if cov is None else np.asarray(cov)

    def pdf(self, x):
        return np.exp(self.logpdf(x))
        
    def logpdf(self, x):
        x    = mvn._process_quantiles(x, self.dim)
        pdf  = mvn(self.mean, self.cov).logpdf(x)
        cdf  = norm(0, 1).logcdf(np.dot(x, self.shape))
        return _squeeze_output(np.log(2) + pdf + cdf)
    
    def visualise(self):
        pos = np.dstack((self._x_range, self._y_range))
        fig = plt.figure()
        ax = fig.add_subplot(111)
        contf = ax.contourf(self._x_range, self._y_range, self._multiplier * self.pdf(pos))
        plt.colorbar(contf, label='Measurement')
        ax.set_xlabel('Easting (m)')
        ax.set_ylabel('Northing (m)')
        plt.show()

class  Sampling_Metric():
    """
    A sampling metric class
    Where the bivariate gaussian distirbution could represent greenhouse gases or temperature
    across a terrain.
    """
    def __init__(self, mu, cov, x_min, x_max, y_min, y_max):
        self._distribution = mvn(mu, cov)
        self._x_range, self._y_range = np.mgrid[x_min:x_max:1, y_min:y_max:1]
        self._multiplier = 1e7

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
    # [[1,0], [0,1]] Normal Unit
    # [[1, 0], [-1, 2]] --> \
    # [[2, 1], [0, 1]] --> -
    # [[2, 1], [0.5, 2]] --> /
    # [[1, 1], [0, 2]] --> |
    # covariance has to be a matrix that is positive semi-definite
    covariance = [[1, 1], [0, 2]]
    covariance = list(np.array(covariance) * range * 100)      # multiplication here explicitly for 5000m x and y length. 
    #stats.multivariate_normal.
    distribution = Sampling_Metric(mean, covariance, x_min, x_max, y_min, y_max)
    print(distribution.sample(mean[0], mean[1]))
    distribution.visualise()
    


