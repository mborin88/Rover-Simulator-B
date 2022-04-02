import numpy as np
import os
import sys
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
import matplotlib.image as mpimg
from PIL import Image
sys.path.append('C:/Users/borin/Documents/GitHub/Rover-Simulator/models')
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


def render2d(terrain_map, cmap='terrain', window_size=(8, 8)):
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
    contf.set_clim(0, 150)      #Map with highest elevation is SX27SW, minus elevation capped to 0, as they are water bodies
    plt.colorbar(contf, label='Elevation (m)')
    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')
    plt.show()
    plt.tight_layout()


def render3d(terrain_map, azimuth=315, altitude=45, cmap='gist_earth', downsample=1,
             window_size=(8, 8)):
    """
    Render a terrain map as a 3d image.
    """
    x_min, y_min = terrain_map.x_llcorner, terrain_map.y_llcorner
    x_max, y_max = x_min + terrain_map.resolution * terrain_map.n_cols, \
                   y_min + terrain_map.resolution * terrain_map.n_rows
    x, y = np.linspace(x_min, x_max, terrain_map.n_cols), \
           np.linspace(y_min, y_max, terrain_map.n_rows)
    xx, yy = np.meshgrid(x, y)
    z = prep_data(terrain_map)
    fig, ax = plt.subplots(subplot_kw=dict(projection='3d'), figsize=window_size)
    ls = LightSource(azimuth, altitude)
    rgb = ls.shade(z, cmap=plt.get_cmap(cmap), vert_exag=0.1, blend_mode='soft')
    ax.plot_surface(xx, yy, z, rstride=downsample, cstride=downsample,
                    facecolors=rgb, linewidth=0, antialiased=False, shade=False)
    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')
    ax.set_zlabel('Elevation (m)')
    ax.grid(False)
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    plt.show()
    plt.tight_layout()


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

def show_rgb(im, ax_range):
    plt.imshow(im, extent=ax_range)
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    plt.show()


if __name__ == '__main__':
    sys.path.append('C:/Users/borin/Documents/GitHub/Rover-Simulator')
    from utils.load_map import *
    map_name = 'SX49SW'
    t_map = read_asc(locate_map(map_name + '_elevation.asc'))
    la_map = read_asc(locate_map(map_name + '_landcover.asc'))
    render2d(t_map)
    #image, axis_range = render_rgb(la_map)
    #show_rgb(image, axis_range)
    #render3d(t_map)

