#when rover is meant to go slow bc of neighbours, x slope just takes rover away

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time
import os
import datetime as dt
import statistics as stats
import random as rand

from models.world import *
from models.slope_physics import *
from models.P_controller import *
from models.pose_logger import *
from utils.load_map import *
from utils.render import *
from utils.path import *


BW = [125, 250, 500]                # Selectable bandwidth, in KHz.
SF = [6, 7, 8, 9, 10, 11, 12]       # Selectable spreading factor.
CR = [4 / 5, 4 / 6, 4 / 7, 4 / 8]   # Selectable coding rate.

# Configure basic simulation settings:
area = 'SP46NE'     # Area to run simulation.
N = 10              # Number of rovers.
rovers_sep = 450          # Distance between rovers, in meter.
x_offset = 475      # Offset from left boundary in easting direction, in meter.
y_offset = 5        # Offset from baseline in northing direction, in meter.
goal_offset = 5     # Of distance to goal is smaller than offset, goal is assumed reached, in meter.
steps = 432000      #432000      # Maximum iteration

t_sampling = 0.1    # Sampling time, in second.
len_interval = 80   # Number of time slots between transmissions for one device.

# Configure communication settings:
user_f = 869.525  # Carrier center frequency, in MHz.
user_bw = BW[0]   # Bandwidth, in kHz.
user_sf = SF[3]   # Spreading factor.
user_cr = CR[3]   # Coding rate.
user_txpw = 24    # Transmitting power, in dBm.

# Configure control settings:
Q = None                                      # State noise.
R = None                                        # Measurement noise.
seed_value = dt.datetime.now().microsecond      #Seed value for noise 
rand.seed(seed_value)

ctrl_policy = 2
# Control policy:
# 0 - meaning no controller;

# 1 - meaning goal-driven controller, if used:
K_goal = [1e-1, 1e-2]  # Control gain for goal-driven controller;

# 2 - meaning passive-cooperative controller, if used:
K_neighbour = [0, 1e-1]  # Control gain for passive-cooperative controller;
decay = 'quad'
zero_crossing = 20 * len_interval #25 communication cycles for it to fully decay

# Log control First bit is raw data, 2nd bit = Summary Data 3rd bit = Graph
log_control = '111'
log_step_interval = 600         #600 steps is 60 seconds which is 1 minute
log_title_tag = "Full Run"
log_title = log_title_tag + ', ' +str(dt.datetime.now())[:-7].replace(':', '-')
log_notes = '''Full Run'''            #Additional notes to be added to Log file if wished

waypoint_interval = 18000  #Log every 30 minutes = 18000 steps
init_waypoints = []
num_of_waypoints = 10

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

    
    image, axis_range = render_rgb(map_landcover)
    fig0 = show_rgb_waypoints(image, axis_range, init_waypoints, x_offset, y_offset, \
        goal_offset, rovers_sep, N, num_of_waypoints)

    for i in range(len(init_waypoints)):
        for j in range(len(init_waypoints[i])):
            init_waypoints[i][j] = init_waypoints[i][j][:2]

    
    # Add rovers to the world.
    for i in range(N):
        world.add_rover(init_waypoints[i][0][0], init_waypoints[i][0][1], init_waypoints[i], q_noise=Q, r_noise=R, num_rovers=N,\
                            decay_type= decay, decay_zero_crossing = zero_crossing)

    # Configure rovers' settings.
    for starter in world.rovers:
        starter.config_radio(user_f, user_bw, user_sf, user_cr, user_txpw)
        starter.radio.set_swarm_size(N)
        starter.radio.set_interval(len_interval)
        starter.radio.set_t_slot(t_sampling)

        # Configure motion logger.
        starter.config_pose_logger(PoseLogger(starter))

        # Set goal point for each rover.
        starter.set_current_goal(starter.waypoints[1])

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
        elif ctrl_policy == 3:
            speed_controller = PController(None, K_neighbour)
            starter.config_speed_controller(speed_controller)
            # The reference for passive-cooperative controller
            # dynamically changes when new packet from the neighbour is received.
            starter.config_control_policy('Simple Passive-cooperative')

    # Step simulation and record data.
    ee = []  # To record formation error.
    step = 0
    while True:
        world.step()
        for l in range(N):
            world.rovers[l].pose_logger.log_pose()
            world.rovers[l].pose_logger.log_velocity()
            world.rovers[l].pose_logger.log_connectivity()

        error = 0.0
        for m in range(N - 1):  # Root mean square formation error
            error += (world.rovers[m + 1].pose_logger.y_pose[-1]
                      - world.rovers[m].pose_logger.y_pose[-1]) ** 2
        ee.append(sqrt(error / (N - 1)))
        step += 1

        invalid_rov_pos = False
        for n in range(N):
            if(world.rovers[n].landcover_termination):
                invalid_rov_pos = True
                break
        
        termination_reason = -1
        if world.completed_rovers == N:
            termination_reason = 0
            break
        elif invalid_rov_pos:
            termination_reason = 1
            break
        elif steps is not None:
            if step == steps:
                termination_reason = 2
                break
        

    # Simulation running time.
    end = time.time()

    # Print logged information.
    print('')
    print('=' * 50)
    print('Time elapse: {} (s)'.format(str(round(world.time, 1))))
    print('=' * 50)
    print('Motion information: ')
    print('\nMax RMSE: {} (m) @ {}s'.format(str(round(max(ee), 2)), str(round(ee.index(max(ee))*t_sampling, 2))))
    print('Mean RMSE: {} (m)'.format(str(round(stats.mean(ee), 2))))
    for k in range(N):
        logger = world.rovers[k].pose_logger
        print('-' * 50)
        print('Rover ID: {}'.format(str(k + 1)))
        print('Distance marched in northing: {} (m)'.format(str(round(logger.y_pose[-1] - logger.y_pose[0]))))
        print('Average speed in northing: {} (m/s)'.format(str(round(sum(logger.velocity) / len(logger.velocity), 2))))
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
    if termination_reason == -1:
        termination_note = "Unknown"
    elif termination_reason == 0:
        termination_note = "Mission Completed"
    elif termination_reason == 1:
        termination_note = "Rover Entered Water Body"
    elif termination_reason == 2:
        termination_note = "Maximum Time Limit Reached"

    print("Termination reason:", termination_note)
    print('Simulation running time: {} (s)'.format(str(round(end - start, 1))))

    #Logs directory creation if not created
    if(int(log_control) >= 1):
        if(not os.path.exists('logs\\' + area)):
            os.mkdir(os.getcwd() + '\\logs\\' + area)
        if(not os.path.exists('logs\\' + area + '\\control_policy_' + str(ctrl_policy))):
            os.mkdir(os.getcwd() + '\\logs\\' + area+ '\\control_policy_' + str(ctrl_policy))
        if(not os.path.exists('logs\\' + area + '\\control_policy_' + str(ctrl_policy) + '\\' + str(log_title))):
            os.mkdir(os.getcwd() + '\\logs\\' + str(area) + '\\control_policy_' + str(ctrl_policy) + '\\' + str(log_title))
        directory = 'logs\\' + str(area) + '\\control_policy_' + str(ctrl_policy) + '\\' + str(log_title) + '\\'

    #Log Summary Information
    if(int(log_control[1]) == 1):
        log_summary_file_name = 'SSS Summary Data'
        log_summary_file = open(directory + log_summary_file_name+'.txt', 'w')
        log_summary_file.write(log_summary_file_name + ': ' + log_title + '\n')
        log_summary_file.write("\nNotes: " + log_notes)
        log_summary_file.write('\n')
        log_summary_file.write('=' * 50)
        log_summary_file.write('\nParameters:\n')
        log_summary_file.write('''Area = {}\nFrequency = {}\nBandwidth(BW) = {}\nSpreading Factor(SF) = {}\nCoding Rate(CR) = {}
            \nTransmitting Power(TxPW) = {}\nRovers(N) = {}\nControl Policy(ctrl_policy) = {}\nDecay Type = {}\nDecay Zero Crossing = {}\nNoise Seed = {}\nState Noise(Q) = {}
            \nMeasurement Noise(R) = {}\nDistance between Rovers(dist) = {}\nX Offset = {}\nY Offset = {}\nGoal Offset = {}
            \nSteps = {}\nMax Steps = {}\nLength Interval = {}\nGoal Driven Gain = {}\nPassive Controller Gain = {}'''\
            .format(str(area), str(user_f), str(user_bw), str(user_sf), str(user_cr), str(user_txpw), str(N), str(ctrl_policy), str(decay), str(zero_crossing),\
                    str(seed_value), str(Q), str(R), str(rovers_sep), str(x_offset), str(y_offset), str(goal_offset), str(step), str(steps), \
                    str(len_interval), str(K_goal), str(K_neighbour)))
        log_summary_file.write('\n')
        log_summary_file.write('=' * 50)
        log_summary_file.write('\n')
        log_summary_file.write('=' * 50)
        log_summary_file.write('\nTime elapse: {} (s)'.format(str(round(world.time, 1))))
        log_summary_file.write('\n')
        log_summary_file.write('=' * 50)
        log_summary_file.write('\nMotion information: ')
        log_summary_file.write('\nMax RMSE: {} (m) @ {}s'.format(str(round(max(ee), 2)), str(round(ee.index(max(ee))*t_sampling, 2))))
        log_summary_file.write('\nMean RMSE: {} (m)'.format(str(round(stats.mean(ee), 2))))
        for k in range(N):
            logger = world.rovers[k].pose_logger
            log_summary_file.write('\n')
            log_summary_file.write('-' * 50)
            log_summary_file.write('\nRover ID: {}'.format(str(k + 1)))
            log_summary_file.write('\nDistance marched in northing: {} (m)'.format(str(round(logger.y_pose[-1] - logger.y_pose[0]))))
            log_summary_file.write('\nAverage speed in northing: {} (m/s)'.format(str(round(sum(logger.velocity) / len(logger.velocity), 2))))
            if logger.termination_time is None:
                log_summary_file.write('\nTask not completed.')
            else:
                log_summary_file.write('\nTime to Complete the Task: {} (s)'.format(str(round(logger.termination_time, 1))))
        log_summary_file.write('\n')
        log_summary_file.write('=' * 50)
        log_summary_file.write('\nCommunication performance: ')
        for j in range(N):
            transceiver = world.rovers[j].radio
            log_summary_file.write('\n')
            log_summary_file.write('-' * 50)
            log_summary_file.write('\nRover ID: {}'.format(str(j + 1)))
            log_summary_file.write('\nSwarm Size: {}'.format(str(transceiver.total_radios)))
            if transceiver is None:
                log_summary_file.write('\nNo radio settings.')
            else:
                log_summary_file.write('\nBandwidth: {} (KHz)'.format(str(transceiver.bw)))
                log_summary_file.write('\nSpreading Factor: {}'.format(str(transceiver.sf)))
                log_summary_file.write('\nCoding Rate: {}/{}'.format(str(4), str(int(4 / transceiver.cr))))
                log_summary_file.write('\nSensitivity: {} (dBm)'.format(str(transceiver.sensitivity)))
                log_summary_file.write('\nTransmission Power: {} (dBm)'.format(str(transceiver.tx_pw)))
                log_summary_file.write('\nAntenna Gain: {} (dBi)'.format(str(transceiver.ant_gain)))
                log_summary_file.write('\nPayload Length: {} (byte)'.format(str(transceiver.pl)))
                log_summary_file.write('\nDuty Cycle: {}%'.format(str(round(transceiver.actual_dc() * 100, 1))))
                log_summary_file.write('\nAirtime: {} (sec)'.format(str(round(transceiver.airtime(), 4))))
                log_summary_file.write('\nSilent time: {} (sec)'.format(str(round(transceiver.actual_silent_time(), 1))))
                log_summary_file.write('\nTransmitted Packets: {}'.format(str(transceiver.num_tx)))
                log_summary_file.write('\nReceived Packets: {}'.format(str(transceiver.num_rx)))
                log_summary_file.write('\nDiscarded Packets: {}'.format(str(transceiver.num_disc)))
                log_summary_file.write('\nPacket Loss Ratio: {}%'.format(str(round(transceiver.num_disc
                                                                / (transceiver.num_rx + transceiver.num_disc) * 100, 2))))
        log_summary_file.write('\n')                                                        
        log_summary_file.write('=' * 50)

        # Log simulation running time.
        log_summary_file.write('\n')
        log_summary_file.write("Termination reason: " + termination_note)
        log_summary_file.write('\nSimulation running time: {} (s)'.format(str(round(end - start, 1))))
        log_summary_file.close()
    
    #Log Raw Data into a file
    if(int(log_control[0]) == 1):
        log_raw_file_name = 'SSS Raw Data'
        log_raw_file = open(directory + log_raw_file_name+'.txt', 'w')
        log_raw_file.write(log_raw_file_name + ': ' + log_title + '\n')
        log_raw_file.write("Notes: " + log_notes + '\n')
        log_raw_file.write('=' * 50)
        log_raw_file.write('\nParameters:\n')
        log_raw_file.write('''Area = {}\nFrequency = {}\nBandwidth(BW) = {}\nSpreading Factor(SF) = {}\nCoding Rate(CR) = {}
            \nTransmitting Power(TxPW) = {}\nRovers(N) = {}\nControl Policy(ctrl_policy) = {}\nDecay Type = {}\nDecay Zero Crossing = {}\nNoise Seed = {}\nState Noise(Q) = {}
            \nMeasurement Noise(R) = {}\nDistance between Rovers(rovers_sep) = {}\nX Offset = {}\nY Offset = {}\nGoal Offset = {}
            \nSteps = {}\nMax Steps = {}\nLength Interval = {}\nLog Interval = {}\nTime Sampling = {}\nGoal Driven Gain = {}\nPassive Controller Gain = {}'''\
            .format(str(area), str(user_f), str(user_bw), str(user_sf), str(user_cr), str(user_txpw), str(N), str(ctrl_policy), str(decay), str(zero_crossing),\
                    str(seed_value) ,str(Q), str(R), str(rovers_sep), str(x_offset), str(y_offset), str(goal_offset), str(step), str(steps), str(len_interval), \
                    str(log_step_interval), str(t_sampling),str(K_goal), str(K_neighbour)))
        log_raw_file.write('\n')
        log_raw_file.write('=' * 50)
        log_raw_file.write("\t\t Rover\n")
        log_raw_file.write("Time\t")
        for x in range(N):
            log_raw_file.write(str(x+1) + 'x\t' + str(x+1) + 'y\t' + str(x+1) + 'v\t')
        log_raw_file.write('RMSE EE')

        for n in range(0, step+1, log_step_interval):   #+1 for velocity to calculate avg speed of the last interval.
            log_raw_file.write('\n' + str(round(n*t_sampling/60, 2)) +'\t')        #divide 60 for per minute
            data = ""
            for j in range(N):
                if(n>=log_step_interval):
                    avg_velocity = round(np.mean(world.rovers[j].pose_logger.velocity[(n-log_step_interval):n]), 3)
                else:
                    avg_velocity = round(world.rovers[j].pose_logger.velocity[n], 3)

                if(n>=step): #need last velocity interval but position index doesn't include it.
                    n = step-1

                data += str(round(world.rovers[j].pose_logger.x_pose[n], 2)) + ',' + str(round(world.rovers[j].pose_logger.y_pose[n], 2)) \
                    + ',' + str(avg_velocity) + '-'

            if(n>=log_step_interval):
                data += str(round(np.mean(ee[n-log_step_interval:n]), 3))
            else:
                data += str(round(np.mean(ee[n]), 3))
            log_raw_file.write(data)
        log_raw_file.close()
        
    # Plot rovers' trajectories over terrain
    x, y = np.linspace(x_min, x_max, map_terrain.n_cols), \
           np.linspace(y_min, y_max, map_terrain.n_rows)
    X, Y = np.meshgrid(x, y)
    Z = prep_data(map_terrain)
    cmap = 'gist_earth'
    
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    contf = ax.contourf(X, Y, Z, cmap=plt.get_cmap(cmap))
    contf.set_clim(0, 375)   #Map with highest elevation is SX27SW, minus elevation capped to 0, as they are water bodies
    plt.colorbar(contf, label='Elevation (m)')

    for o in range(N):
        plotter = world.rovers[o].pose_logger
        ax.plot(plotter.x_pose, plotter.y_pose, linewidth=1.8, color='red')
    
    #Waypoint grapher on contour plot
    for k in range(waypoint_interval, step, waypoint_interval):
        x_waypoint = []
        y_waypoint = []
        for q in range(N):
            plotter = world.rovers[q].pose_logger
            x_waypoint.append(plotter.x_pose[k])
            y_waypoint.append(plotter.y_pose[k])
        ax.plot(x_waypoint, y_waypoint, linewidth=1.8, color='black')

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')
    ax.set_title('Swarm Trajectory (Time Elapse: {} sec)'.format(str(round(world.time, 1))))
    if(int(log_control[2]) == 1):
        fig0.savefig(directory + 'Path_Planned_Trajectory.png', dpi=100)
        plt.savefig(directory + 'Elevation.png')
    
    #RMSE of rovers position error over time
    fig1, ax1 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    ax1.set_ylim(0, 250)
    ax1.set_xlim(0.0, world.time/60)
    avg_ee = []
    for q in range(0, step+1, log_step_interval):
        if(q>=log_step_interval):
            avg_ee.append(round(np.mean(ee[(q-log_step_interval):q]), 2))
        else:
            avg_ee.append(round(ee[q], 2))
    ax1.plot(avg_ee)
    ax1.set_xlabel('Time (min)')
    ax1.set_ylabel('Root Mean Square Formation Error (m)')
    ax1.set_title('Average Collective Formation Performance per minute (Time Elapse: {} min)'.format(str(round(world.time/60, 1))))
    if(int(log_control[2]) == 1):
        plt.savefig(directory + 'RMSE.png')

    #Velocity boxplots of each rover whiskers going from 0% - 100%
    # fig2, ax2 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    # labels = []
    # velocities = []
    # for p in range(N):
    #     v_plotter = world.rovers[p].pose_logger
    #     velocities.append(v_plotter.velocity)
    #     labels.append('ID: ' + str(p + 1))
    # ax2.boxplot(velocities, autorange=True, showfliers=False, whis=(0,100))
    # ax2.set_xticklabels(labels)
    # ax2.set_title('Average Velocity Curve per minute (Time Elapse: {} min)'.format(str(round(world.time/60, 1))))
    # del velocities

    if(int(log_control[2]) == 1):
        plt.savefig(directory + 'Velocity.png')

    # Plot rovers' trajectories over landcover
    fig3, ax3 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    image, axis_range = render_rgb(map_landcover) 
    ax3.imshow(image, extent=axis_range)

    for o in range(N):
        plotter = world.rovers[o].pose_logger
        ax3.plot(plotter.x_pose, plotter.y_pose, linewidth=1.8, color='cyan')
    
    #Waypoint grapher for landcover map
    for k1 in range(waypoint_interval, step, waypoint_interval):
        x1_waypoint = []
        y1_waypoint = []
        for q1 in range(N):
            plotter = world.rovers[q1].pose_logger
            x1_waypoint.append(plotter.x_pose[k1])
            y1_waypoint.append(plotter.y_pose[k1])
        ax3.plot(x1_waypoint, y1_waypoint, linewidth=1.8, color='white')

    ax3.set_xlim(x_min, x_max)
    ax3.set_ylim(y_min, y_max)
    ax3.set_xlabel('Easting (m)')
    ax3.set_ylabel('Northing (m)')
    ax3.set_title('Swarm Trajectory (Time Elapse: {} sec)'.format(str(round(world.time, 1))))
    if(int(log_control[2]) == 1):
        plt.savefig(directory + 'Landcover.png')
    
    #Y Position of each rover time. 
    fig4, ax4 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    ax4.set_ylim(y_min, y_max) 
    ax4.set_xlim(0.0, world.time/60)
    labels = []
    for p in range(N):
        v_plotter = world.rovers[p].pose_logger
        rover_y_pose = []
        for a in range(0, step+1, log_step_interval):
            if(a>=step):
                a = step-1
            rover_y_pose.append(v_plotter.y_pose[a])
        ax4.plot(rover_y_pose, linewidth=1.8)
        labels.append('ID: ' + str(p + 1))

    ax4.legend(labels) 
    ax4.set_xlabel('Time (min)')
    ax4.set_ylabel('Y position (m)')
    ax4.set_title('Y postion Trajectory (Time Elapse: {} min)'.format(str(round(world.time/60, 1))))
    if(int(log_control[2]) == 1):
        plt.savefig(directory + 'Y_Position.png')

    #Overview of connectivity for the mission
    fig5, ax5 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    rover_connectivity = []
    for x in range(N):
        rover_connectivity.append([])
        connectivity_plotter = world.rovers[x].pose_logger
        sum_connectivity = []
        for c_step in range(len_interval, step+1, len_interval):
            if(c_step>=len_interval):
                rover_connectivity_interval = np.array(connectivity_plotter.connectivity[(c_step-len_interval):c_step])
                sum_connectivity.append(sum(np.sum(rover_connectivity_interval, axis=0)))
        rover_connectivity[x] = sum_connectivity
    
    connectivity = []
    for i in range(len(rover_connectivity[0])):
        connectivity_interval = [comm_time[i] for comm_time in rover_connectivity]
        connectivity.append(sum(connectivity_interval)/((N**2)-N))
    
    ax5.set_ylim([-0.1, 1])
    ax5.boxplot(connectivity, autorange=True, showfliers=False, whis=(0,100))
    ax5.set_title('Connectivity of Mission')
    ax5.set_ylabel('Connectivity')
    ax5.set_xlabel('Mission')
    if(int(log_control[2]) == 1):
        plt.savefig(directory + 'Mission_Connectivity.png')


    #Individual connectvity of each rover
    # connectivity_fig = [0]*N
    # connectivity_ax = [0]*N
        
    # for b in range(N):
    #     connectivity_fig[b], connectivity_ax[b] = plt.subplots(nrows=1, ncols=1, figsize=(6, 6)) 
    #     connectivity_ax[b].set_ylim(0, ((log_step_interval/len_interval) + 1)) 
    #     connectivity_ax[b].set_xlim(0.0, world.time/60)
    #     connectivity_ax[b].set_xlabel('Time (min)')
    #     connectivity_ax[b].set_ylabel('Numbers of times connected to in the minute')
    #     connectivity_ax[b].set_title('Connection of rover {} (Time Elapse: {} min)'.format(str(b+1) ,str(round(world.time/60, 1))))

    #     connectivity_plotter = world.rovers[b].pose_logger
    #     sum_connectivity = []
    #     for c_step in range(0, step+1, log_step_interval):
    #         if(c_step>=log_step_interval):
    #             rover_connectivity_interval = np.array(connectivity_plotter.connectivity[(c_step-log_step_interval):c_step])
    #             sum_connectivity.append(np.sum(rover_connectivity_interval, axis=0))
    #         else:
    #             sum_connectivity.append(np.array([0]*N))

    #     labels = []
    #     plot_connectivity = []
    #     for z in range(N):
    #         plot_connectivity.append([item[z] for item in sum_connectivity])
    #         connectivity_ax[b].plot(plot_connectivity[z], linewidth=1.8)
    #         labels.append('ID: ' + str(z + 1))
    #     connectivity_ax[b].legend(labels)
    #     if(int(log_control[2]) == 1):
    #         plt.savefig(directory + 'Connection_of_rover_' + str(b+1) + '.png')

    plt.show()
    plt.tight_layout()

if __name__ == '__main__':
    main()
