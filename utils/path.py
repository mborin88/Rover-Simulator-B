from glob import glob
from tkinter.messagebox import YES
from matplotlib import offsetbox
import numpy as np
import os
import sys
import math
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
import matplotlib.image as mpimg
from PIL import Image
sys.path.append('C:/Users/borin/Documents/GitHub/Rover-Simulator/models')
#from models.landcover_spec import LCM2015_COLORMAP
#from landcover_spec import LCM2015_COLORMAP
from landcover_spec import LCM2015_COLORMAP


def prep_data(map_object):
    """
    Prepare data for rendering.
    """
    data_re = np.zeros((map_object.n_rows, map_object.n_cols))
    for i in range(map_object.n_rows):
        for j in range(map_object.n_cols):
            data_re[i, j] = map_object.data[map_object.n_rows - i - 1, j]
    return data_re

def render_rgb(landcover_map, cmap=LCM2015_COLORMAP):
    """
    Render a land cover map using specified RGB colour scheme.
    """
    rows, cols = landcover_map.n_rows, landcover_map.n_cols
    rgb = Image.new('RGB', (rows, cols))
    for i in range(rows):
        for j in range(cols):
            rgb.putpixel((i, j), (cmap[int(landcover_map.data[i, j])][0],
                                  cmap[int(landcover_map.data[i, j])][1],
                                  cmap[int(landcover_map.data[i, j])][2]))
    rgb.save(os.path.abspath(os.path.dirname(__file__)) + '\\temp.png')
    temp_im = os.path.abspath(os.path.dirname(__file__)) + '\\temp.png'
    im = mpimg.imread(temp_im)
    dx = landcover_map.resolution
    dy = landcover_map.resolution
    x_min, y_min = landcover_map.x_llcorner, landcover_map.y_llcorner
    x_max, y_max = x_min + dx * landcover_map.n_cols, \
                   y_min + dy * landcover_map.n_rows
    ax_range = (x_min, x_max, y_min, y_max)
    os.remove(temp_im)
    return im, ax_range


def render2d_waypoints(terrain_map, cmap='gist_earth', window_size=(8, 8)):
    """
    Render a terrain map as a 2d contour plot.
    """
    x_min, y_min = terrain_map.x_llcorner, terrain_map.y_llcorner
    x_max, y_max = x_min + terrain_map.resolution * terrain_map.n_cols, \
                   y_min + terrain_map.resolution * terrain_map.n_rows
    x, y = np.linspace(x_min, x_max, terrain_map.n_cols), \
           np.linspace(y_min, y_max, terrain_map.n_rows)
    xx, yy = np.meshgrid(x, y)
    z = prep_data(terrain_map)
    fig, ax = plt.subplots(figsize=window_size)
    contf = ax.contourf(xx, yy, z, cmap=plt.get_cmap(cmap))

    offset = 250
    sep = 500
    N = 10
    #using 3D list here however in main program each rover will have its own list so it will
    #reduce to a 2D list
    waypoints = []
    num_waypoints = 10
    y_sep = (y_max- y_min) / (num_waypoints-1)

    for rover in range(N):
        waypoints.append([])
        for w_point in range(num_waypoints): #No. waypoints
            waypoints[rover].append([])
            waypoints[rover][w_point].append(x_min + offset + (rover*sep))
            waypoints[rover][w_point].append(y_min +(w_point*y_sep)) # waypoint difference.
            waypoints[rover][w_point].append(0) # waypoint difference.

    x_plt = [x[0] for r in waypoints for x in r]
    x_plt = [x_plt[i:i+num_waypoints] for i in range(0, len(x_plt), num_waypoints)]
    y_plt = [y[1] for r in waypoints for y in r]
    y_plt = [y_plt[i:i+num_waypoints] for i in range(0, len(y_plt), num_waypoints)]

    for i in range(N):
        ax.plot(x_plt[i], y_plt[i], marker='o', markersize=6, linewidth=1.8, color='black')
    
    plt.colorbar(contf, label='Elevation (m)')
    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')

    global clicked
    clicked = False
    path_info = [0,0]

    def closestWaypoint(mouse_x, mouse_y):
        for i in range(N):
            for j in range(0, num_waypoints):
                waypoints[i][j][2] = math.sqrt((mouse_x-waypoints[i][j][0])**2 + (mouse_y-waypoints[i][j][1])**2)

        smallest = 8000
        rover = -1
        waypoint = -1
        for i in range(N):
            for j in range(num_waypoints):
                if(waypoints[i][j][2] < smallest):
                    smallest = waypoints[i][j][2]
                    rover = i
                    waypoint = j
        return rover, waypoint
                

    def onclick(event):
        global clicked
        if(not clicked):
            path_info[0], path_info[1] = closestWaypoint(event.xdata, event.ydata)
        else:
            waypoints[path_info[0]][path_info[1]][0] = event.xdata 

        x_plt = [x[0] for r in waypoints for x in r]
        x_plt = [x_plt[i:i+num_waypoints] for i in range(0, len(x_plt), num_waypoints)]
        plt.cla()
        ax.contourf(xx, yy, z, cmap=plt.get_cmap(cmap), zorder=1)
        ax.set_xlabel('Easting (m)')
        ax.set_ylabel('Northing (m)')
        for i in range(N):
            ax.plot(x_plt[i], y_plt[i], marker='o', markersize=6, linewidth=1.8, color='black', zorder=2)
        if(not clicked):
            ax.scatter([waypoints[path_info[0]][path_info[1]][0]], [waypoints[path_info[0]][path_info[1]][1]], color='red', s=[50], zorder=3)
            clicked = True
        else:
            clicked = False
        plt.draw()

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    plt.tight_layout()

def show_rgb(im, ax_range):
    plt.imshow(im, extent=ax_range)
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    plt.show()


if __name__ == '__main__':
    sys.path.append('C:/Users/borin/Documents/GitHub/Rover-Simulator')
    from utils.load_map import *
    t_map = read_asc(locate_map('TL16NE_elevation.asc'))
    # la_map = read_asc(locate_map('SU20NE_landcover.asc'))
    render2d_waypoints(t_map)
    plt.close()
    # image, axis_range = render_rgb(la_map)
    # show_rgb(image, axis_range)
    #render3d(t_map)

