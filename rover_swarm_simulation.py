import numpy as np
import matplotlib.pyplot as plt
import time

from models.world import *
from models.slope_physics import *
from models.P_controller import *
from models.pose_logger import *
from utils.load_map import *
from utils.render import *


BW = [125, 250, 500]                # Selectable bandwidth, in KHz.
SF = [6, 7, 8, 9, 10, 11, 12]       # Selectable spreading factor.
CR = [4 / 5, 4 / 6, 4 / 7, 4 / 8]   # Selectable coding rate.

# Configure basic simulation settings:
area = 'SU20NE'     # Area to run simulation.
N = 10              # Number of rovers.
dist = 450          # Distance between rovers, in meter.
x_offset = 475      # Offset from left boundary in easting direction, in meter.
y_offset = 5        # Offset from baseline in northing direction, in meter.
goal_offset = 5     # Of distance to goal is smaller than offset, goal is assumed reached, in meter.
steps = 100         #432000      # Maximum iteration.
t_sampling = 0.1    # Sampling time, in second.
len_interval = 50   # Number of time slots between transmissions for one device.

# Configure communication settings:
user_f = 869.525  # Carrier center frequency, in MHz.
user_bw = BW[0]   # Bandwidth, in kHz.
user_sf = SF[3]   # Spreading factor.
user_cr = CR[3]   # Coding rate.
user_txpw = 24    # Transmitting power, in dBm.

# Configure control settings:
Q = None         # State noise.
R = None           # Measurement noise.
ctrl_policy = 2
# Control policy:
# 0 - meaning no controller;

# 1 - meaning goal-driven controller, if used:
K_goal = [0, 1e-2]  # Control gain for goal-driven controller;

# 2 - meaning passive-cooperative controller, if used:
K_neighbour = [0, 1]  # Control gain for passive-cooperative controller;


def main():
    """
    The very first simulation.
    """
    print('')
    print('Simulating...')

    start = time.time()

    # Load terrain map and land cover map to create the world.
    # Configure world's dynamics engine.
    map_terrain = read_asc(locate_map(area + '_elevation' + '.asc'))
    map_landcover = read_asc(locate_map(area + '_landcover' + '.asc'))
    x_min, x_max = map_terrain.x_llcorner, map_terrain.x_llcorner + map_terrain.x_range
    y_min, y_max = map_terrain.y_llcorner, map_terrain.y_llcorner + map_terrain.y_range
    world = World(map_terrain, map_landcover, t_sampling)
    world.config_engine(SlopePhysics(world))

    # Add rovers to the world.
    for i in range(N):
        world.add_rover(x_min + x_offset + i * dist, y_min + y_offset, q_noise=Q, r_noise=R)

    # Configure rovers' settings.
    for starter in world.rovers:
        starter.config_radio(user_f, user_bw, user_sf, user_cr, user_txpw)
        starter.radio.set_swarm_size(N)
        starter.radio.set_interval(len_interval)
        starter.radio.set_t_slot(t_sampling)

        # Configure motion logger.
        starter.config_pose_logger(PoseLogger(starter))

        # Set goal point for each rover.
        starter.set_goal([starter.pose[0], y_max - goal_offset])

        # Configure controller.
        if ctrl_policy == 0:  # No controller
            pass
        elif ctrl_policy == 1:  # Goal-driven controller
            speed_controller = PController(None, K_goal)
            # The reference is goal point [x_g, y_g],
            # which is set differently for each rover.
            starter.config_speed_controller(speed_controller)
            starter.speed_controller.set_ref(starter.goal)
            starter.config_control_policy('Goal-driven')
        elif ctrl_policy == 2:  # Passive-cooperative controller
            speed_controller = PController(None, K_neighbour)
            starter.config_speed_controller(speed_controller)
            # The reference for passive-cooperative controller
            # dynamically changes when new packet from the neighbour is received.
            starter.config_control_policy('Passive-cooperative')

    # Step simulation and record data.
    ee = []  # To record formation error.
    step = 0
    while True:
        world.step()
        for l in range(N):
            world.rovers[l].pose_logger.log_pose()
            world.rovers[l].pose_logger.log_velocity()
        error = 0.0
        for m in range(N - 1):  # Root mean square formation error
            error += (world.rovers[m + 1].pose_logger.y_pose[-1]
                      - world.rovers[m].pose_logger.y_pose[-1]) ** 2
        ee.append(sqrt(error / (N - 1)))
        step += 1
        if world.completed_rovers == N:
            break
        elif steps is not None:
            if step == steps:
                break

    # Simulation running time.
    end = time.time()

    # Print logged information.
    print('')
    print('=' * 50)
    print('Time elapse: {} (s)'.format(str(round(world.time, 1))))
    print('=' * 50)
    print('Motion information: ')
    for k in range(N):
        logger = world.rovers[k].pose_logger
        print('-' * 50)
        print('Rover ID: {}'.format(str(k + 1)))
        print('Distance marched in northing: {} (m)'.format(str(round(logger.y_pose[-1] - logger.y_pose[0]))))
        if logger.termination_time is None:
            print('Task not completed.')
        else:
            print('Time to Complete the Task: {} (s)'.format(str(round(logger.termination_time, 1))))
    print('=' * 50)
    print('Communication performance: ')
    for j in range(N):
        transceiver = world.rovers[j].radio
        print('-' * 50)
        print('Rover ID: {}'.format(str(j + 1)))
        print('Swarm Size: {}'.format(str(transceiver.total_radios)))
        if transceiver is None:
            print('No radio settings.')
        else:
            print('Bandwidth: {} (KHz)'.format(str(transceiver.bw)))
            print('Spreading Factor: {}'.format(str(transceiver.sf)))
            print('Coding Rate: {}/{}'.format(str(4), str(int(4 / transceiver.cr))))
            print('Sensitivity: {} (dBm)'.format(str(transceiver.sensitivity)))
            print('Transmission Power: {} (dBm)'.format(str(transceiver.tx_pw)))
            print('Antenna Gain: {} (dBi)'.format(str(transceiver.ant_gain)))
            print('Payload Length: {} (byte)'.format(str(transceiver.pl)))
            print('Duty Cycle: {}%'.format(str(round(transceiver.actual_dc() * 100, 1))))
            print('Airtime: {} (sec)'.format(str(round(transceiver.airtime(), 4))))
            print('Silent time: {} (sec)'.format(str(round(transceiver.actual_silent_time(), 1))))
            print('Transmitted Packets: {}'.format(str(transceiver.num_tx)))
            print('Received Packets: {}'.format(str(transceiver.num_rx)))
            print('Discarded Packets: {}'.format(str(transceiver.num_disc)))
            print('Packet Loss Ratio: {}%'.format(str(round(transceiver.num_disc
                                                            / (transceiver.num_rx + transceiver.num_disc) * 100, 2))))
    print('=' * 50)

    # Print simulation running time.
    print('')
    print('Simulation running time: {} (s)'.format(str(round(end - start, 1))))

    # Plot rovers' trajectories.
    x, y = np.linspace(x_min, x_max, map_terrain.n_cols), \
           np.linspace(y_min, y_max, map_terrain.n_rows)
    X, Y = np.meshgrid(x, y)
    Z = prep_data(map_terrain)
    cmap = 'gist_earth'

    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    contf = ax.contourf(X, Y, Z, cmap=plt.get_cmap(cmap))
    plt.colorbar(contf, label='Elevation (m)')
    # labels = []
    for o in range(N):
        plotter = world.rovers[o].pose_logger
        ax.plot(plotter.x_pose, plotter.y_pose, linewidth=1.8, color='red')
    #    labels.append('ID: ' + str(o + 1))
    # ax.legend(labels)
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')
    ax.set_title('Swarm Trajectory (Time Elapse: {} sec)'.format(str(round(world.time, 1))))

    fig1, ax1 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    ax1.plot(ee)
    ax1.set_xlim(0.0, world.time)
    ax1.set_xlabel('Time (sec)')
    ax1.set_ylabel('Root Mean Square Formation Error (m)')
    ax1.set_title('Collective Formation Performance (Time Elapse: {} sec)'.format(str(round(world.time, 1))))

    fig2, ax2 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    labels = []
    for p in range(N):
        v_plotter = world.rovers[p].pose_logger
        ax2.plot(v_plotter.velocity, linewidth=1.8)
        labels.append('ID: ' + str(p + 1))
    ax2.legend(labels)
    ax2.set_xlim(0.0, world.time)
    ax2.set_xlabel('Time (sec)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Velocity Curve (Time Elapse: {} sec)'.format(str(round(world.time, 1))))

    plt.show()
    plt.tight_layout()


if __name__ == '__main__':
    main()
