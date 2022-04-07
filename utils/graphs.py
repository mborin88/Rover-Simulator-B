from tkinter import W
import numpy as np
import matplotlib.pyplot as plt
from utils.render import *


def terrain_plot(world, control_policy, map_terrain, x_min, x_max, y_min, y_max, N, waypoint_interval, step, graph_log, directory):
    """
    Plots and logs the rover trajectories against terrain map with line test at every waypoint interval
    Also plots and logs user planned path
    """
    x, y = np.linspace(x_min, x_max, map_terrain.n_cols), \
           np.linspace(y_min, y_max, map_terrain.n_rows)
    X, Y = np.meshgrid(x, y)
    Z = prep_data(map_terrain)
    cmap = 'gist_earth'
    
    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    contf = ax.contourf(X, Y, Z, cmap=plt.get_cmap(cmap))
    contf.set_clim(0, 150)   #Map with highest elevation is 150, minus elevation capped to 0, as they are water bodies
    plt.colorbar(contf, label='Elevation (m)')

    for o in range(N):
        plotter = world.rovers[o].pose_logger
        ax.plot(plotter.x_pose, plotter.y_pose, linewidth=1.8, color='red')

    if(control_policy == 4):
        samples = []
        for i in range(N):
            samples.append(world.rovers[i]._measured_samples)
        x = [data[0] for waypoint in samples for data in waypoint]
        y = [data[1] for waypoint in samples for data in waypoint]
        ax.scatter(x, y, 20, color='black')
    elif(control_policy != 4):
        # Waypoint grapher on contour plot
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
    if(int(graph_log) == 1):
        plt.savefig(directory + 'Elevation.png')


def RMSE_plot(world, step, log_step_interval, ee, graph_log, directory):
    """"
    Plots RMSE over time with upper limit set so that the graphs are comparable.
    """
    upper_limit = 150
    fig1, ax1 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    ax1.set_ylim(0, upper_limit)
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
    if(int(graph_log) == 1):
        plt.savefig(directory + 'RMSE.png')


def velocity_plot(world, N, graph_log, directory):
    """
    Plots magnitude of each rovers velocity as a box plot.
    """
    fig2, ax2 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    labels = []
    velocities = []
    for p in range(N):
        v_plotter = world.rovers[p].pose_logger
        velocities.append(v_plotter.velocity)
        labels.append('ID: ' + str(p + 1))
    ax2.boxplot(velocities, autorange=True, showfliers=False, whis=(0,100))
    ax2.set_xticklabels(labels)
    ax2.set_title('Average Velocity Curve per minute (Time Elapse: {} min)'.format(str(round(world.time/60, 1))))
    del velocities

    if(int(graph_log) == 1):
        plt.savefig(directory + 'Velocity.png')
    

def landcover_plot(world, control_policy, map_landcover, x_min, x_max, y_min, y_max, N, waypoint_interval, step, graph_log, directory):
    """
    Plots and logs the rover trajectories agaisnt landcover map with line test at every waypoint interval 
    """
    fig3, ax3 = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
    image, axis_range = render_rgb(map_landcover) 
    ax3.imshow(image, extent=axis_range)

    for o in range(N):
        plotter = world.rovers[o].pose_logger
        ax3.plot(plotter.x_pose, plotter.y_pose, linewidth=1.8, color='cyan')
    
    if(control_policy == 4):
        samples = []
        for i in range(N):
            samples.append(world.rovers[i]._measured_samples)
        x = [data[0] for waypoint in samples for data in waypoint]
        y = [data[1] for waypoint in samples for data in waypoint]
        ax3.scatter(x, y, 20, color='white')
    elif(control_policy != 4):
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
    if(int(graph_log) == 1):
        plt.savefig(directory + 'Landcover.png')


def y_position_plot(world, step, log_step_interval, y_min, y_max, N, graph_log, directory):
    """
    Plots y position of each rover against time. 
    """
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
    if(int(graph_log) == 1):
        plt.savefig(directory + 'Y_Position.png')


def mission_connectivity_plot(world, N, len_interval, step, graph_log, directory):
    """
    Overall mission connecctivity box plot.
    Shows spread of connectivity thorughout misssion.
    1 = All rovers were able to communicate to all rover.
    0 = No rovers were able to communicate to each other
    """
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
    if(int(graph_log) == 1):
        plt.savefig(directory + 'Mission_Connectivity.png')

def individual_rover_connectivty(world, step, log_step_interval, len_interval, N, graph_log, directory):
    """
    Individual connectvity of each rover 
    Specifies which rover each rover is connected to at every time step
    """
    connectivity_fig = [0]*N
    connectivity_ax = [0]*N
        
    for b in range(N):
        connectivity_fig[b], connectivity_ax[b] = plt.subplots(nrows=1, ncols=1, figsize=(6, 6)) 
        connectivity_ax[b].set_ylim(0, ((log_step_interval/len_interval) + 1)) 
        connectivity_ax[b].set_xlim(0.0, world.time/60)
        connectivity_ax[b].set_xlabel('Time (min)')
        connectivity_ax[b].set_ylabel('Numbers of times connected to in the minute')
        connectivity_ax[b].set_title('Connection of rover {} (Time Elapse: {} min)'.format(str(b+1) ,str(round(world.time/60, 1))))

        connectivity_plotter = world.rovers[b].pose_logger
        sum_connectivity = []
        for c_step in range(0, step+1, log_step_interval):
            if(c_step>=log_step_interval):
                rover_connectivity_interval = np.array(connectivity_plotter.connectivity[(c_step-log_step_interval):c_step])
                sum_connectivity.append(np.sum(rover_connectivity_interval, axis=0))
            else:
                sum_connectivity.append(np.array([0]*N))

        labels = []
        plot_connectivity = []
        for z in range(N):
            plot_connectivity.append([item[z] for item in sum_connectivity])
            connectivity_ax[b].plot(plot_connectivity[z], linewidth=1.8)
            labels.append('ID: ' + str(z + 1))
        connectivity_ax[b].legend(labels)
        if(int(graph_log) == 1):
            plt.savefig(directory + 'Connection_of_rover_' + str(b+1) + '.png')

def generate_distribution(world, N, x_min, x_max, y_min, y_max, directory, graph_log):
    samples = []
    for i in range(N):
        samples.append(world.rovers[i]._measured_samples)

    x = [data[0] for waypoint in samples for data in waypoint]
    y = [data[1] for waypoint in samples for data in waypoint]
    metric = [data[2] for waypoint in samples for data in waypoint]

    try:
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
        cmap = 'viridis'
        contf = ax.tricontourf(x, y, metric, cmap=plt.get_cmap(cmap))
        # contf.set_clim(0, 150)
        plt.colorbar(contf, label='Measurment')
        plt.xlim([x_min, x_max])                        # Change to using the sampling distribution object from world
        plt.ylim([y_min, y_max])
        if(int(graph_log) == 1):
            plt.savefig(directory + 'Sampled Measurements.png')
    except RuntimeError:
        print("Can't infer a graph from the data")

def real_metric_distribution(world, directory, graph_log):
    if(world._sample_metric._mean is not None):
        pos = np.dstack((world._sample_metric._x_range, world._sample_metric._y_range))
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
        contf = ax.contourf(world._sample_metric._x_range, world._sample_metric._y_range, \
                world._sample_metric._multiplier * world._sample_metric.distribution.pdf(pos))
        plt.colorbar(contf, label='Measurement')
        ax.set_xlabel('Easting (m)')
        ax.set_ylabel('Northing (m)')
        if(int(graph_log) == 1):
            plt.savefig(directory + 'Real Metric Distribution.png')