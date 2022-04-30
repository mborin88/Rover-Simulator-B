#Generation of sampling metric
#Could repreent greenhouse gasses, temperature, moisture...
#options were to keep between -1:1:.01 and then scale the position to between here
import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from  scipy.stats import (multivariate_normal as mvn,norm)
from  scipy.stats._multivariate import _squeeze_output

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
        self._mean  = mean
        self._x_min = x_min
        self._x_max = x_max
        self._y_min = y_min
        self._y_max = y_max
        self._x_range, self._y_range = np.mgrid[x_min:x_max:1, y_min:y_max:1]
        self.cov   = np.eye(self.dim) if cov is None else np.asarray(cov)

    def pdf(self, x):
        return np.exp(self.logpdf(x))
        
    def logpdf(self, x):
        x    = mvn._process_quantiles(x, self.dim)
        pdf  = mvn(self._mean, self.cov).logpdf(x)
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
    def __init__(self, x_min, x_max, y_min, y_max):
        self._x_min = x_min
        self._x_max = x_max
        self._y_min = y_min
        self._y_max = y_max
        self._mean  = None                                   # Mean of metric value
        self._covariance = []                                   # Covariance of metric distribution (has to be positive semidefinite)
        self._distribution = None
        self._x_range, self._y_range = np.mgrid[x_min:x_max:1, y_min:y_max:1]
        self._multiplier = 1e8


    @property
    def distribution(self):
        return self._distribution

    @property
    def mean(self):
        return self._mean

    @property
    def covariance(self):
        return self._covariance

    def config_distribution(self):
        if(self._mean is not None):
            self._distribution = mvn(self._mean, self._covariance)


    def sample(self, x, y):
        if(self._mean is not None):
            return self._multiplier * self.distribution.pdf([x, y])


    def visualise(self):
        pos = np.dstack((self._x_range, self._y_range))
        fig = plt.figure()
        ax = fig.add_subplot(111)
        contf = ax.contourf(self._x_range, self._y_range, self._multiplier * self.pdf(pos))
        plt.colorbar(contf, label='Measurement')
        ax.set_xlabel('Easting (m)')
        ax.set_ylabel('Northing (m)')
        ax.set_aspect('equal', 'box')
        plt.show()


    def visualise_overlay(self, terrain_map):
        if(self._mean is not None):
            x_min, y_min = terrain_map.x_llcorner, terrain_map.y_llcorner
            x_max, y_max = x_min + terrain_map.resolution * terrain_map.n_cols, \
                        y_min + terrain_map.resolution * terrain_map.n_rows
            x, y = np.linspace(x_min, x_max, terrain_map.n_cols), \
                np.linspace(y_min, y_max, terrain_map.n_rows)
            xx, yy = np.meshgrid(x, y)
            z = prep_data(terrain_map)
            fig, ax = plt.subplots(figsize=(7,7))
            contf = ax.contourf(xx, yy, z, cmap=plt.get_cmap('gist_earth'))
            contf.set_clim(0, 150)      #Map with highest elevation is SX27SW, minus elevation capped to 0, as they are water bodies
            pos = np.dstack((self._x_range, self._y_range))
            contf1 = ax.contourf(self._x_range, self._y_range, self._multiplier * self.distribution.pdf(pos), cmap='YlOrBr', alpha=0.4)
            ax.set_aspect('equal', 'box')

            plt.colorbar(contf1, label='Measurement', shrink=0.85)
            plt.colorbar(contf, label='Elevation (m)', shrink=0.85)
            ax.set_xlabel('Easting (m)')
            ax.set_ylabel('Northing (m)')
            fig.set_size_inches(8,6)
            plt.savefig('temp.png', dpi=300)
            plt.show()
            
            plt.tight_layout()
    

    def config_mean(self, pseudo_mean):
        mean_x_flag = True
        if(pseudo_mean[0] == 'M'):
            mean_x = int((self._x_min + self._x_max)/2)
        elif(pseudo_mean[0] == 'R'):
            mean_x = self._x_min + int(3*(self._x_max - self._x_min)/4)
        elif(pseudo_mean[0] == 'L'):
            mean_x = self._x_min + int(1*(self._x_max - self._x_min)/4)
        else:
            print("Invalid mean selection")
            mean_x_flag = False
        
        mean_y_flag = True
        if(pseudo_mean[1] == 'M'):
            mean_y = int((self._y_min + self._y_max)/2)
        elif(pseudo_mean[1] == 'T'):
            mean_y = self._y_min + int(3*(self._y_max - self._y_min)/4)
        elif(pseudo_mean[1] == 'B'):
            mean_y = self._y_min + int(1*(self._y_max - self._y_min)/4)
        else:
            print("Invalid mean selection")
            mean_y_flag = False
        
        if(mean_x_flag == True and mean_y_flag == True):
            self._mean = [mean_x, mean_y]


    def config_covariance(self, psuedo_covariance):
        range = self._x_max - self._x_min
        self._covariance = list(np.array(psuedo_covariance) * range * 100)      # multiplication here explicitly for 5000m x and y length. 
        

if __name__ == '__main__':
    sys.path.append(str(os.getcwd()))
    from utils.load_map import *
    from utils.render import *
    area = 'SU20NE'
    map_terrain = read_asc(locate_map(area + '_elevation' + '.asc'))
    map_landcover = read_asc(locate_map(area + '_landcover' + '.asc'))
    x_min, x_max = map_terrain.x_llcorner, map_terrain.x_llcorner + map_terrain.x_range
    y_min, y_max = map_terrain.y_llcorner, map_terrain.y_llcorner + map_terrain.y_range

    mean = ['R', 'M']
    # [[1,0], [0,1]] Normal Unit
    # [[1, 0], [-1, 2]] --> \
    # [[2, 1], [0, 1]] --> -
    # [[2, 1], [0.5, 2]] --> / [[3, 1], [1, 1]], [[2, 1], [1, 1]]
    # [[1, 1], [0, 2]] --> |
    # covariance has to be a matrix that is positive semi-definite
    covariance = [[1, 0], [0, 1]]
    distribution = Sampling_Metric(x_min, x_max, y_min, y_max)
    distribution.config_mean(mean)
    distribution.config_covariance(covariance)
    distribution.config_distribution()
    #print(distribution.sample(mean[0], mean[1]))
    distribution.visualise_overlay(map_terrain)