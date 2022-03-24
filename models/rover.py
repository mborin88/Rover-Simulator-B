from msilib import RadioButtonGroup
from random import *
import statistics as stats
import numpy as np
import math

from models.radio import *
from models.P_controller import *

STARTING_SPEED = 0.2       # m/s
MAXIMUM_SPEED = 0.5        # m/s, which can be exceeded due to the effect of slope.
MINIMUM_SPEED = 0.05       # m/s, which depicts the worst scenario and cannot be decreased any more.


class Rover:
    """
    A rover class.
    decay_type = 'quad'
    zero_cross = 1200 #Decay = 0 at 2 minutes of silence from that rover.
    """
    def __init__(self, rov_id, easting, northing, rov_waypoints, q_noise=None, r_noise=None, num_rovers=10, decay_type = 'quad', decay_zero_crossing=1200):
        self._rov_id = rov_id                 # Unique id for each rover.
        self._pose = [easting, northing]      # Pose: (x (m), y (m)).
        self._angle = 0
        self._q_noise = q_noise               # The state noise, a random variable.
        self._r_noise = r_noise               # The measurement noise, a random variable.
        self.measurement = self._pose         # The measurement of pose, assumed noiseless at first.
        self._waypoints = rov_waypoints

        self._control = [0, STARTING_SPEED, STARTING_SPEED]      # Control input, linear velocity.
        self._all_control = np.array([np.nan] * (num_rovers + 1)) #First control is p control then rovers
        self._steps_control_not_updated = np.array([0.0] * (num_rovers + 1)) #Amount of steps since that control has been updated.
        self._num_rovers = num_rovers
        self._initial_control = True          # Want to P controller until we get neighbouring positions
        self._control_policy = None
        self._decay_type = decay_type
        self._decay_zero_crossing = decay_zero_crossing
        self._connectivity = [0] * num_rovers

        # The control policy used by the rover.
        self._speed_controller = None
        # The speed controller, a specific type of controller object.
        self._radio = None              # The communication module, a radio object.
        self._goal_index = 1
        self._termination_flag = False  # The flag indicating that mission is terminated.
        self._termination_time = None   # The time when rover completes its task.
        self.pose_logger = None         # The logger to record pose, a motion logger object.

    @property
    def rov_id(self):
        return self._rov_id

    @property
    def pose(self):
        return self._pose
    
    @property
    def angle(self):
        return self._angle

    @property
    def q_noise(self):
        return self._q_noise

    @property
    def r_noise(self):
        return self._r_noise
    
    @property
    def waypoints(self):
        return self._waypoints
    
    @property
    def goal_index(self):
        return self._goal_index

    @property
    def control(self):
        return self._control
    
    @property
    def all_control(self):
        return self._all_control
    
    @property
    def steps_control_not_updated(self):
        return self._steps_control_not_updated

    @property
    def num_rovers(self):
        return self._num_rovers

    @property
    def initial_control(self):
        return self._initial_control

    @property
    def control_policy(self):
        return self._control_policy

    @property
    def decay_type(self):
        return self._decay_type

    @property
    def decay_zero_crossing(self):
        return self._decay_zero_crossing

    @property
    def connectivity(self):
        return self._connectivity

    @property
    def speed_controller(self):
        return self._speed_controller

    @property
    def radio(self):
        return self._radio

    @property
    def goal(self):
        return self._waypoints[self._goal_index]

    @property
    def termination_time(self):
        return self._termination_time

    def config_radio(self, f, bw, sf, cr, tx_pw):
        """
        Configure radio parameters.
        """
        self._radio = Radio(self, f, bw, sf, cr, tx_pw)

    def config_control_policy(self, policy):
        """
        Configure control policy.
        """
        self._control_policy = policy

    def config_speed_controller(self, controller):
        """
        Configure the type of controller used to control velocity.
        """
        self._speed_controller = controller

    def config_pose_logger(self, logger):
        """
        Configure pose logger.
        """
        self.pose_logger = logger

    def set_q_noise(self, new_q):
        """
        Set state noise.
        """
        self._q_noise = new_q

    def set_r_noise(self, new_r):
        """
        Set measurement noise.
        """
        self._r_noise = new_r

    def generate_noise(self, sigma):
        """
        Generate noise, assuming zero mean.
        """
        gen_noise = []
        for i in range(len(sigma)):
            gen_noise.append(gauss(0, sigma[i]))
        return gen_noise

    def set_current_goal(self, goal):
        """
        Set goal point.
        """
        self._current_goal = goal

    def update_speeds(self, vx, vy):
        self._control[0] = round(vx, 3)
        self._control[1] = round(vy, 3)
        self._control[2] = round(sqrt(vx ** 2 + vy ** 2), 3)  # Update speed.

    def step_motion(self, world, dt):
        """
        Simulate rover's motion through the given time interval.
        """
        if self.is_mission_terminated():
            pass
        else:
            p = self._pose
            u = self._control
            slope_y = world.dynamics_engine.northing_slope(p[0], p[1])
            a_y = world.dynamics_engine.generate_acceleration(slope_y)
            a_yf = world.dynamics_engine.generate_friction(slope_y)
            v_y = u[1] + a_y * dt + a_yf * dt

            slope_x = world.dynamics_engine.easting_slope(p[0], p[1])
            a_x = world.dynamics_engine.generate_acceleration(slope_x)
            a_xf = world.dynamics_engine.generate_friction(slope_x)
            v_x = u[0] + a_x * dt + a_xf * dt
            
            if v_y < MINIMUM_SPEED:
                v_y = MINIMUM_SPEED

            # Assume in the worst scenario rover can still maintain a minimum speed.
            h = [p[0] + dt * v_x,
                 p[1] + dt * v_y]  # State transition.
            if h[1] < world.terrain.y_llcorner:
                h[1] = world.terrain.y_llcorner
                print('Cannot back off behind the baseline.')
            elif h[1] >= (world.terrain.y_llcorner + world.terrain.y_range):
                h[1] = world.terrain.y_llcorner + world.terrain.y_range - 1e-6
                print('Cannot move beyond the upper boundary.')
            elif h[0] < world.terrain.x_llcorner:
                h[0] = world.terrain.x_llcorner
                print('Cannot go off right side of map.')
            elif h[0] >= (world.terrain.x_llcorner + world.terrain.x_range):
                h[0] = world.terrain.x_llcorner + world.terrain.x_range - 1e-6
                print('Cannot go off left side of map')
            if self._q_noise is None:
                self._pose[0] = h[0]
                self._pose[1] = h[1]  # Noiseless motion.
                self.update_speeds(v_x, v_y)
                self.measure()
            else:
                noise = self.generate_noise(self._q_noise)
                new_pose = [h[0] + noise[0], h[1] + noise[1]]
                if((new_pose[1] > world.terrain.y_llcorner) and (new_pose[1] < (world.terrain.y_llcorner + world.terrain.y_range))):
                    self._pose[1] = new_pose[1]  # Noisy motion.
                else:
                    self._pose[1] = h[1]
                
                if(new_pose[0] > world.terrain.x_llcorner and new_pose[0] < (world.terrain.x_llcorner + world.terrain.x_range)):
                    self._pose[0] = new_pose[0]  #No noise on x yet
                else:
                    self._pose[0] = h[0]
                self.update_speeds(v_x, v_y)
                self.measure()

    def halt(self):
        """
        Discard any ongoing action and halt immediately.
        """
        self._control[2] = 0
    
    def x_multiplier(self, value):
        if(value >= 0):
            return 1
        else:
            return -1
    
    def ratio_speeds(self):
        p = self._pose
        target = self.goal
        v = self._control[2]
        x_diff = target[0] - p[0]
        y_diff = target[1] - p[1]
        angle = math.atan(y_diff/abs(x_diff))
        self._angle = angle
        
        self._control[0] = round(self.x_multiplier(x_diff) * v * math.cos(angle), 3)
        self._control[1] = round(v * math.sin(angle), 3)

    def apply_control(self, world):
        """
        Apply control effect to the rover.
        """
        if self.is_mission_terminated():
            pass
        else:
            if self.is_final_goal_reached():
                self.halt()
                self.terminate()
                self.terminate_in_world(world)
                self.pose_logger.log_termination_time()
            else:
                if self._speed_controller is None:
                    pass
                elif self._control_policy is None:
                    pass
                elif self._control_policy == 'Goal-driven':
                    self.move2goal()
                elif self._control_policy == 'Passive-cooperative':
                    self.passive_cooperation()
                elif self._control_policy == 'Simple Passive-cooperative':
                    self.simple_passive_cooperation()

    def move2goal(self):
        """
        Move towards goal point.
        """
        offset = 5

        controlled_object = self.measurement  # Controlled object is [x, y].
        if(self._pose[1] > self.goal[1]-offset):   #if within offset of the y waypoint
            self._goal_index += 1
            self.speed_controller.set_ref(self.goal)
        control_input = self._speed_controller.execute2(controlled_object)

        if control_input > MAXIMUM_SPEED:  # Control input saturation.
            self._control[2] = MAXIMUM_SPEED
        elif control_input < MINIMUM_SPEED:
            self._control[2] = MINIMUM_SPEED
        else:
            self._control[2] = control_input  # Assume changing linear velocity instantly.

        self.ratio_speeds()


    def passive_cooperation(self):
        """
        Apply passive cooperative control, i.e. only adjust speed when neighbour(s)' info.
        Recieve all info scale old info to be less relevant. At certain time thershold old ignore old info.
        Slowly push all_control values that haven't been recieved to 0.
        """

        goal_driven_controller = PController(ref=self._current_goal, gain=[0, 1e-2])
        controlled_object = self.measurement
        control_input = goal_driven_controller.execute2(controlled_object)
        
        if control_input > MAXIMUM_SPEED:  # Control input saturation.
            p_control = MAXIMUM_SPEED
        elif control_input < MINIMUM_SPEED:
            p_control = MINIMUM_SPEED
        else:
            p_control = control_input  # Assume changing linear velocity instantly. #velocity changes at end of cooperation 

        self._control[2] = p_control
        self.ratio_speeds()
        self._all_control[0] = self._control[1]     #only want y speed to do line sweeping with
        x_mult = self.x_multiplier(self._control[0])

        neighbour_poses = self.get_neighbour_pose()
        if neighbour_poses.count(None) < self._num_rovers:
            self._initial_control = False

        self.connectivity_reset()
        self.neighbour_connectivity(neighbour_poses)
        
        for i in range(1, len(self._steps_control_not_updated)):
            self._steps_control_not_updated[i] += 1 #all incremented by 1

        if not self._initial_control:
            # Adjust speed according to neighbour(s)' info.
            if neighbour_poses.count(None) < self._num_rovers:
                control_index = 0
                for pose in neighbour_poses:
                    control_index += 1
                    if pose is not None:
                        self._steps_control_not_updated[control_index] = 0
                        self._speed_controller.set_ref(pose)
                        controlled_object = self.measurement
                        self._all_control[control_index] = self._speed_controller.execute(controlled_object)
                self.scale_all_control()
                control_input = self.weighted_control_calc()
            else:
                self.scale_all_control()
                control_input = self.weighted_control_calc()
        else:
            control_input = self._all_control[0] # Take 100% portion of goal_driven control.
        
        if control_input > self._control[1]:  # Control input saturation. Only control the vy
            self._control[1] = self._control[1]
        elif control_input < MINIMUM_SPEED:     #Test out with 0 next but could leave dead in the water
            self._control[1] = MINIMUM_SPEED
        else:
            self._control[1] = control_input  # Assume changing linear velocity instantly.

        self.update_speeds(x_mult * self.control[1] / math.tan(self._angle), self._control[1])    

        self._radio.reset_neighbour_register()
        self._radio.reset_buffer()
    
    def simple_passive_cooperation(self):
        """
        Apply passive cooperative control, i.e. only adjust speed when neighbour(s)' info is received,
        otherwise do not apply any control effect.
        Start with P controller then only change speed when neighbour info recieved again.
        """

        goal_driven_controller = PController(ref=self._current_goal, gain=[1e-2, 1e-2])
        controlled_object = self.measurement
        control_input = goal_driven_controller.execute2(controlled_object)
        
        if control_input > MAXIMUM_SPEED:  # Control input saturation.
            p_control = MAXIMUM_SPEED
        elif control_input < MINIMUM_SPEED:
            p_control = MINIMUM_SPEED
        else:
            p_control = control_input  # Assume changing linear velocity instantly. #velocity changes at end of cooperation 

        self._all_control[0] = p_control

        neighbour_poses = self.get_neighbour_pose()
        if neighbour_poses.count(None) < self._num_rovers:
            self._initial_control = False

        self.connectivity_reset()
        self.neighbour_connectivity(neighbour_poses)
        

        for i in range(1, len(self._steps_control_not_updated)):
            self._steps_control_not_updated[i] += 1 #all incremented by 1

        if not self._initial_control:
            # Adjust speed according to neighbour(s)' info.
            if neighbour_poses.count(None) < self._num_rovers:
                control_index = 0
                for pose in neighbour_poses:
                    control_index += 1
                    if pose is not None:
                        self._speed_controller.set_ref(pose)
                        controlled_object = self.measurement
                        self._all_control[control_index] = self._speed_controller.execute(controlled_object)
                        self._steps_control_not_updated[control_index] = 0 
                control_input = self.weighted_control_calc()
                
            else:
                control_input = self.weighted_control_calc()
        else:
            control_input = p_control*1  # Take 100% portion of goal_driven control.
        
        if control_input > self._control[1]:  # Control input saturation.
            self._control[1] = self._control[1]
        elif control_input < MINIMUM_SPEED:
            self._control[1] = MINIMUM_SPEED
        else:
            self._control[1] = control_input  # Assume changing linear velocity instantly.    

        self._radio.reset_neighbour_register()
        self._radio.reset_buffer()
    
    def weighted_control_calc(self):
        """
        Mean control of all the neighbouring velocities
        Then summed with the P controller Speed.
        """ 
        mean_control = 1
        if(mean_control):
            neighbour_mean = np.nanmean(self._all_control[1:])
            return self._all_control[0] + neighbour_mean

    def time_decay(self, value):
        if(self._decay_type == 'exp'):
            scale = (-1/np.exp(-value + self._decay_zero_crossing)) + 1
        elif (self._decay_type == 'quad'): 
            scale = (-1/(self._decay_zero_crossing**2)) * (value-self._decay_zero_crossing) * (value + self._decay_zero_crossing)
        else:
            scale = 1

        if(scale <= 0):
            scale = 0

        return scale

    def scale_all_control(self):
        """
        Scale all control variable. 
        """ 
        multiplier = np.array([1.0]*len(self._all_control))
        for i in range(1,len(self._all_control)):
            multiplier[i] = self.time_decay(self._steps_control_not_updated[i])
        self._all_control = self._all_control * multiplier

    def measure(self):
        """
        Measure the pose info at current time.
        """
        if self._r_noise is None:
            self.measurement = self._pose  # Noiseless measurement.
        else:
            noise = self.generate_noise(self._r_noise)
            self.measurement[0] = self._pose[0] + noise[0]
            self.measurement[1] = self._pose[1] + noise[1]  # Noisy measurement.

    def connectivity_reset(self):
        """
        Reset connectivity array
        """
        self._connectivity = [0]*self._num_rovers

    def neighbour_connectivity(self, neighbours):
        """
        Reset connectivity array
        """
        for i in range(len(neighbours)):
            if neighbours[i] is not None:
                self._connectivity[i] = 1
            else:
                self._connectivity[i] = 0

    def is_final_goal_reached(self):
        """
        See if the goal point is reached.
        """
        return self._pose[1] >= self._waypoints[-1][1]

    def terminate(self):
        """
        Terminate the global mission and remove all the noises.
        """
        self._termination_flag = True
        # If rover exceeds the goal point, still set the current state and measurement
        # to goal point in order to facilitate other rovers to complete their tasks.
        self._pose = self._current_goal
        self.measurement = self._current_goal

        if self._q_noise is not None:
            self._q_noise = None

        if self._r_noise is not None:
            self._r_noise = None

    def terminate_in_world(self, world):
        """
        Let the world know the rover has terminated.
        """
        world.rover_completes_task()
        self._termination_time = world.time
        print('Rover ID {} has completed the task.'.format(str(self._rov_id)))

    def is_mission_terminated(self):
        """
        See if the mission is terminated.
        """
        return self._termination_flag

    def get_neighbour_info(self):
        """
        Get neighbour(s)' info from the packet received by radio.
        """
        return self._radio.neighbour_register

    def get_neighbour_pose(self):
        """
        Get neighbour(s)' pose info.
        """
        neighbour_info = self.get_neighbour_info()
        poses = []
        for info in neighbour_info:
            if info is None:
                poses.append(None)
            else:
                poses.append(info.payload[2:])
        return poses

    def get_interval(self):
        """
        Get interval from radio.
        """
        return self._radio.interval

    def get_swarm_size(self):
        """
        Get swarm size from radio.
        """
        return self._radio.total_radios
