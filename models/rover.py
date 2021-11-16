from random import *
import statistics as stats

from models.radio import *
from models.P_controller import *

STARTING_SPEED = 0.2       # m/s
MAXIMUM_SPEED = 0.5        # m/s, which can be exceeded due to the effect of slope.
MINIMUM_SPEED = 0.05       # m/s, which depicts the worst scenario and cannot be decreased any more.


class Rover:
    """
    A rover class.
    """
    def __init__(self, rov_id, easting, northing, q_noise=None, r_noise=None):
        self._rov_id = rov_id                 # Unique id for each rover.
        self._pose = [easting, northing]      # Pose: (x (m), y (m)).
        self._q_noise = q_noise               # The state noise, a random variable.
        self._r_noise = r_noise               # The measurement noise, a random variable.
        self.measurement = self._pose         # The measurement of pose, assumed noiseless at first.
        self._control = [STARTING_SPEED]      # Control input, linear velocity.
        self._old_control = [STARTING_SPEED]  # Old control velocity applied
        self._control_policy = None
        # The control policy used by the rover.
        self._speed_controller = None
        # The speed controller, a specific type of controller object.
        self._radio = None              # The communication module, a radio object.
        self._goal = None               # The goal point desired to be reached.
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
    def q_noise(self):
        return self._q_noise

    @property
    def r_noise(self):
        return self._r_noise

    @property
    def control(self):
        return self._control
    
    @property
    def old_control(self):
        return self._old_control

    @property
    def control_policy(self):
        return self._control_policy

    @property
    def speed_controller(self):
        return self._speed_controller

    @property
    def radio(self):
        return self._radio

    @property
    def goal(self):
        return self._goal

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

    def set_goal(self, goal):
        """
        Set goal point.
        """
        self._goal = goal

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
            a = world.dynamics_engine.generate_acceleration(slope_y)
            a_f = world.dynamics_engine.generate_friction(slope_y)
            v_x = 0
            v_y = u[0] + a * dt + a_f * dt
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
            if self._q_noise is None:
                self._pose[0] = h[0]
                self._pose[1] = h[1]  # Noiseless motion.
                self._old_control[0] = self._control[0]
                self._control[0] = sqrt(v_x ** 2 + v_y ** 2)  # Update speed.
                self.measure()
            else:
                noise = self.generate_noise(self._q_noise)
                self._pose[0] = h[0] + noise[0]
                self._pose[1] = h[1] + noise[1]  # Noisy motion.
                self._old_control[0] = self._control[0]
                self._control[0] = sqrt(v_x ** 2 + v_y ** 2)  # Update speed.
                self.measure()

    def halt(self):
        """
        Discard any ongoing action and halt immediately.
        """
        self._control[0] = 0

    def apply_control(self, world):
        """
        Apply control effect to the rover.
        """
        if self.is_mission_terminated():
            pass
        else:
            if self.is_goal_reached():
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

    def move2goal(self):
        """
        Move towards goal point.
        """
        controlled_object = self.measurement  # Controlled object is [x, y].
        control_input = self._speed_controller.execute(controlled_object)
        if control_input > MAXIMUM_SPEED:  # Control input saturation.
            self._control[0] = MAXIMUM_SPEED
        elif control_input < MINIMUM_SPEED:
            self._control[0] = MINIMUM_SPEED
        else:
            self._control[0] = control_input  # Assume changing linear velocity instantly.

    def passive_cooperation(self):
        """
        Apply passive cooperative control, i.e. only adjust speed when neighbour(s)' info is received,
        otherwise do not apply any control effect.
        """

        goal_driven_controller = PController(ref=self._goal, gain=[0, 1e-4])
        controlled_object = self.measurement
        control_input = goal_driven_controller.execute(controlled_object)
        p_control = 0
        
        if control_input > MAXIMUM_SPEED:  # Control input saturation.
            p_control = MAXIMUM_SPEED
        elif control_input < MINIMUM_SPEED:
            p_control = MINIMUM_SPEED
        else:
            p_control = control_input  # Assume changing linear velocity instantly. #velocity changes at end of cooperation
        #self._control[0] *= 1  # Take 100% portion of goal_driven control. #Ignore

        passive_control = [self._old_control[0]]

        # Adjust speed according to neighbour(s)' info.
        neighbour_poses = self.get_neighbour_pose()
        for pose in neighbour_poses:
            if pose is not None:
                passive_control = []

        for pose in neighbour_poses:
            coop_control = 0
            if pose is not None:
                self._speed_controller.set_ref(pose)
                controlled_object = self.measurement
                coop_control = self._speed_controller.execute(controlled_object)
                # print(coop_control)
                control_input = p_control + coop_control 
                if control_input > MAXIMUM_SPEED:  # Control input saturation.
                    passive_control.append(MAXIMUM_SPEED)
                elif control_input < MINIMUM_SPEED:
                    passive_control.append(MINIMUM_SPEED)
                else:
                    passive_control.append(control_input)  # Assume changing linear velocity instantly.

        avg_passive_control = stats.mean(passive_control)
        self._control[0] = (p_control + avg_passive_control) / 2

        self._radio.reset_neighbour_register()
        self._radio.reset_buffer()

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

    def is_goal_reached(self):
        """
        See if the goal point is reached.
        """
        return self._pose[1] >= self._goal[1]

    def terminate(self):
        """
        Terminate the global mission and remove all the noises.
        """
        self._termination_flag = True
        # If rover exceeds the goal point, still set the current state and measurement
        # to goal point in order to facilitate other rovers to complete their tasks.
        self._pose = self._goal
        self.measurement = self._goal

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
