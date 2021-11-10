from random import *
from numpy import *
import matplotlib.pyplot as plt

from models.world import *
from utils.load_map import *


BW = [125, 250, 500]                # Selectable bandwidth, in KHz.
SF = [6, 7, 8, 9, 10, 11, 12]       # Selectable spreading factor.
CR = [4 / 5, 4 / 6, 4 / 7, 4 / 8]   # Selectable coding rate.

user_f = 869.525  # Carrier center frequency, in MHz.
user_bw = BW[0]   # Bandwidth, in kHz.
user_sf = SF[3]   # Spreading factor.
user_cr = CR[3]   # Coding rate.
user_txpw = 24    # Transmitting power, in dBm.

area = 'SU20NE'  # Area to test path loss.
dist = 450       # Distance between two points.
max_rep = 10000  # Number of times the test is repeated.


def main():
    """
    Test path loss between two random points which are on a same horizontal line at a given distance
    repeating for certain number of times.
    """
    name_terrain = area + '_elevation' + '.asc'
    name_landcover = area + '_landcover' + '.asc'
    map_terrain = read_asc(locate_map(name_terrain))
    map_landcover = read_asc(locate_map(name_landcover))
    world = World(map_terrain, map_landcover)
    x_min, y_min = world.terrain.x_llcorner, world.terrain.y_llcorner
    x_range, y_range = map_terrain.x_range, map_terrain.y_range
    x_max, y_max = x_min + x_range, y_min + y_range

    test_x = [randint(x_min, x_max - 1) for _ in range(max_rep)]
    test_y = [randint(y_min, y_max - 1) for _ in range(max_rep)]
    path_loss = []
    for i in range(max_rep):
        x_test_1 = test_x[i]
        y_test_1 = test_y[i]

        if (x_test_1 + dist) >= x_max:
            x_test_2 = x_test_1 - dist
        else:
            x_test_2 = x_test_1 + dist

        y_test_2 = y_test_1

        test_points = [[x_test_1, y_test_1], [x_test_2, y_test_2]]
        for point in test_points:
            world.add_rover(point[0], point[1])

        for rover in world.rovers:
            rover.config_radio(user_f, user_bw, user_sf, user_cr, user_txpw)

        loss_model = PathLoss(world.rovers[0].radio, world.rovers[1].radio, world)
        loss = loss_model.total_loss()
        path_loss.append(loss)
        show_test_points = [[int(test_points[0][0]), int(test_points[0][1])],
                            [int(test_points[1][0]), int(test_points[1][1])]]
        print('Test No.{} Position {} Path Loss: {} (dB)'.format(str(i + 1), show_test_points,
                                                                 str(round(loss, 2))))

        world.rovers.clear()

    max_loss = max(path_loss)
    min_loss = min(path_loss)
    mean_loss = mean(path_loss)

    print()
    print('Maximum Path Loss: {} (dB)'.format(str(max_loss)))
    print('Minimum Path Loss: {} (dB)'.format(str(min_loss)))
    print('Average Path Loss: {} (dB)'.format(str(mean_loss)))

    plt.plot(path_loss)
    plt.xlabel('Number of Test')
    plt.ylabel('Path Loss (dB)')
    plt.title('Distance between test points: {} (m)'.format(str(dist)))
    plt.show()


if __name__ == '__main__':
    main()
