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

def show_rgb_waypoints(im, ax_range, waypoints, x_offset, y_offset, goal_offset, r_sep, N, num_waypoints):
    fig, ax = plt.subplots(figsize=(6, 6))

    #using 3D list here however in main program each rover will have its own list so it will
    #reduce to a 2D list
    y_sep = (ax_range[3]- ax_range[2]) / (num_waypoints-1)

    for rover in range(N):
        waypoints.append([])
        for w_point in range(num_waypoints): #No. waypoints
            waypoints[rover].append([])
            waypoints[rover][w_point].append(ax_range[0] + x_offset + (rover*r_sep))
            if(w_point==0):
                waypoints[rover][w_point].append(ax_range[2] + y_offset +(w_point*y_sep)) # waypoint difference.
            elif(w_point == num_waypoints):
                waypoints[rover][w_point].append(ax_range[3] - goal_offset)
            else:
                waypoints[rover][w_point].append(ax_range[2] +(w_point*y_sep))
            waypoints[rover][w_point].append(0) # waypoint difference.

    x_plt = [x[0] for r in waypoints for x in r]
    x_plt = [x_plt[i:i+num_waypoints] for i in range(0, len(x_plt), num_waypoints)]
    y_plt = [y[1] for r in waypoints for y in r]
    y_plt = [y_plt[i:i+num_waypoints] for i in range(0, len(y_plt), num_waypoints)]

    for i in range(N):
        ax.plot(x_plt[i], y_plt[i], marker='o', markersize=6, linewidth=1.8, color='white')
    
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
            path_info[0], path_info[1] = closestWaypoint(round(event.xdata), round(event.ydata))
        else:
            waypoints[path_info[0]][path_info[1]][0] = round(event.xdata)

        x_plt = [x[0] for r in waypoints for x in r]
        x_plt = [x_plt[i:i+num_waypoints] for i in range(0, len(x_plt), num_waypoints)]
        plt.cla()
        ax.imshow(im, extent=ax_range)
        ax.set_xlabel('Easting (m)')
        ax.set_ylabel('Northing (m)')
        for i in range(N):
            ax.plot(x_plt[i], y_plt[i], marker='o', markersize=6, linewidth=1.8, color='white', zorder=2)
        if(not clicked):
            ax.scatter([waypoints[path_info[0]][path_info[1]][0]], [waypoints[path_info[0]][path_info[1]][1]], color='cyan', s=[50], zorder=3)
            clicked = True
        else:
            clicked = False
        plt.draw()

    fig.canvas.mpl_connect('button_press_event', onclick)

    ax.imshow(im, extent=ax_range)
    plt.show()
    plt.ioff()


if __name__ == '__main__':
    sys.path.append('C:/Users/borin/Documents/GitHub/Rover-Simulator')
    from utils.load_map import *
    la_map = read_asc(locate_map('SU20NE_landcover.asc'))
    image, axis_range = render_rgb(la_map)
    waypoints = []
    N = 10
    rovers_sep = 450          # Distance between rovers, in meter.
    x_off = 475      # Offset from left boundary in easting direction, in meter.
    y_off = 5  
    g_off = 5  
    num_of_waypoints = 10
    show_rgb_waypoints(image, axis_range, waypoints, x_off, y_off, g_off, rovers_sep, N, num_of_waypoints)

