import numpy as np
import os
import sys
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
import re

sys.path.append(str(os.getcwd()) + '\\models')
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
            rgb.putpixel((i, j), (cmap[int(landcover_map.data[j, i])][0],
                                  cmap[int(landcover_map.data[j, i])][1],
                                  cmap[int(landcover_map.data[j, i])][2]))
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

def get_waypoints(file, loaded_waypoints):
    """
    Grabs and formats the waypoints of another simulation misssion.
    Only missions from the parameters file work.
    """
    my_file = open(file, "r")
    data = my_file.readlines()
    data = data[43:]
    for i in range(len(data)):
        index = data[i].find('[')
        data[i] = data[i][index:]
        data[i] = [int(s) for s in re.findall(r'-?\d+\.?\d*', data[i])]
    
    for i in range(len(data)):
        loaded_waypoints.append([])
        for j in range(0, len(data[i]), 2):
            loaded_waypoints[i].append([])
            loaded_waypoints[i][int(j/2)].append(data[i][j])
            loaded_waypoints[i][int(j/2)].append(data[i][j+1])
            loaded_waypoints[i][int(j/2)].append(0)


def show_rgb_waypoints(im, ax_range, waypoints, load, mission, x_offset, y_offset, goal_offset, r_sep, N, num_waypoints):
    fig0, ax0 = plt.subplots(figsize=(6, 6))

    #using 3D list here however in main program each rover will have its own list so it will
    #reduce to a 2D list
    y_sep = (ax_range[3]- ax_range[2]) / (num_waypoints-1)
    
    if(load == True):
        pass
    else:
        for rover in range(N):
            waypoints.append([])
            for w_point in range(num_waypoints): #No. waypoints
                waypoints[rover].append([])
                waypoints[rover][w_point].append(round(ax_range[0] + x_offset + (rover*r_sep)))
                if(w_point==0):
                    waypoints[rover][w_point].append(round(ax_range[2] + y_offset +(w_point*y_sep))) # waypoint difference.
                elif(w_point == num_waypoints):
                    waypoints[rover][w_point].append(round(ax_range[3] - goal_offset))
                else:
                    waypoints[rover][w_point].append(round(ax_range[2] +(w_point*y_sep)))
                waypoints[rover][w_point].append(0) # waypoint difference.
    
    if(mission != 'LS'):

        x_plt = [x[0] for r in waypoints for x in r]
        x_plt = [x_plt[i:i+num_waypoints] for i in range(0, len(x_plt), num_waypoints)]
        y_plt = [y[1] for r in waypoints for y in r]
        y_plt = [y_plt[i:i+num_waypoints] for i in range(0, len(y_plt), num_waypoints)]

        for i in range(N):
            ax0.plot(x_plt[i], y_plt[i], marker='o', markersize=6, linewidth=2, color='white')
        
        ax0.set_xlabel('Easting (m)')
        ax0.set_ylabel('Northing (m)')

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
                try:
                    path_info[0], path_info[1] = closestWaypoint(round(event.xdata), round(event.ydata))
                except TypeError:
                    print("Click was outside the map")
            else:
                waypoints[path_info[0]][path_info[1]][0] = round(event.xdata)

            x_plt = [x[0] for r in waypoints for x in r]
            x_plt = [x_plt[i:i+num_waypoints] for i in range(0, len(x_plt), num_waypoints)]
            plt.cla()
            ax0.imshow(im, extent=ax_range)
            ax0.set_xlabel('Easting (m)')
            ax0.set_ylabel('Northing (m)')
            for i in range(N):
                ax0.plot(x_plt[i], y_plt[i], marker='o', markersize=6, linewidth=2, color='white', zorder=2)
            if(not clicked):
                ax0.scatter([waypoints[path_info[0]][path_info[1]][0]], [waypoints[path_info[0]][path_info[1]][1]], color='cyan', s=25, zorder=3)
                clicked = True
            else:
                clicked = False
            plt.draw()

        fig0.canvas.mpl_connect('button_press_event', onclick)

        ax0.imshow(im, extent=ax_range)

        path_fig0 = plt.gcf()
        plt.show()
        plt.ioff()
        return path_fig0
    return 0


if __name__ == '__main__':
    sys.path.append(os.getcwd())
    from utils.load_map import *
    la_map = read_asc(locate_map('SU20NE_landcover.asc'))
    image, axis_range = render_rgb(la_map)
    waypoints = []
    N = 10
    rovers_sep = 450          # Distance between rovers, in meter.
    x_off = 475      # Offset from left boundary in easting direction, in meter.
    y_off = 5  
    g_off = 5  
    load_points = 0
    num_of_waypoints = 10
    show_rgb_waypoints(image, axis_range, waypoints, load_points, x_off, y_off, g_off, rovers_sep, N, num_of_waypoints)
    #waypoints = get_waypoints('temp.txt')
