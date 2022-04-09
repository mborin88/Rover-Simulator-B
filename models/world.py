from exceptions.map_not_aligned import *
from models.rover import *


class World:
    """
    A world class.
    """
    def __init__(self, map_terrain, map_landcover, mission='LS', dt=0.1):  # Input two map objects and a time step in ms.
        if self.is_aligned(map_terrain, map_landcover):
            self._terrain = map_terrain                     # Terrain info, a map object.
            self._landcover = map_landcover                 # Land cover info, a map object.
            self._sample_metric = None                       
            self._tn = 0                                    # Simulation time, represented by a sequence order.
            self._dt = dt                                   # Step time, in second.
            self._mission = mission
            self._rovers = []                               # List of existing rovers.
            self.channel = []                               # List of existing transmissions.
            self._dynamics_engine = None                    # Dynamics engine.
            self._completed_rovers = 0                      # Number of rovers that have completed their tasks.
        else:
            raise MapNotAligned()

    @property
    def terrain(self):
        return self._terrain

    @property
    def landcover(self):
        return self._landcover

    @property
    def sample_metric(self):
        return self._sample_metric

    @property
    def time(self):
        return self._tn * self._dt

    @property
    def tn(self):
        return self._tn

    @property
    def dt(self):
        return self._dt
    
    @property
    def mission(self):
        return self._mission

    @property
    def rovers(self):
        return self._rovers

    @property
    def dynamics_engine(self):
        return self._dynamics_engine

    @property
    def completed_rovers(self):
        return self._completed_rovers

    def config_engine(self, engine):
        """
        Configure dynamics engine of the world.
        """
        self._dynamics_engine = engine
    
    def config_sample_metric(self, distribution, mu, cov):
        """
        Configure Sampling Metric and it's distribution in the world.
        """
        self._sample_metric = distribution
        self._sample_metric.config_mean(mu)
        self._sample_metric.config_covariance(cov)
        self._sample_metric.config_distribution()

    def is_aligned(self, map_1, map_2):
        """
        See if two maps are aligned.
        Accept two map objects as input.
        """
        if (map_1.x_llcorner == map_2.x_llcorner) and (map_1.y_llcorner == map_2.y_llcorner):
            if map_1.resolution == map_2.resolution:
                if (map_1.n_cols == map_2.n_cols) and (map_1.n_rows == map_2.n_rows):
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False

    def add_rover(self, easting, northing, r_path, s_dist, q_noise=None, r_noise=None, num_rovers=10,\
                        decay_type='quad', decay_zero_crossing=1200):
        """
        Add a rover to the world given its coordinates.
        """
        self._rovers.append(Rover(len(self.rovers) + 1, easting, northing, r_path, s_dist, q_noise, r_noise, num_rovers,\
                                    decay_type, decay_zero_crossing))

    def rover_completes_task(self):
        """
        Add the count of rovers that have completed their tasks.
        """
        self._completed_rovers += 1

    def add_packet(self, packet):
        """
        Add a new transmission to the channel.
        """
        self.channel.append(packet)

    def clear_channel(self):
        """
        Empty the channel.
        """
        self.channel.clear()

    def step(self):
        """
        Step forward one time interval in simulation.
        """
        dt = self._dt
        tn = self._tn
        if((dt*tn)%500 == 0):
            print('Time: {} (s)\n'.format(str(round(tn * dt, 1))))

        # Logically, this is the beginning of time slot.
        if(self._mission == 'LS'):
            transmitter = None
            for rover in self._rovers:
                rover.step_motion(self, dt)
                if rover.radio is None:
                    pass
                else:
                    if tn == rover.radio.next_tx:
                        transmitter = rover
                        transmitter.radio.transmit(self)
        elif(self._mission):
            transmitter = None
            for rover in self._rovers:
                rover.step_motion(self, dt)
                if rover.radio is None:
                    pass
                else:
                    if tn == rover.radio.next_tx:
                        transmitter = rover
                        transmitter.radio.transmit(self)            

        #Slowing down simulation
        #if len(self.channel) > 0:
        #    print('Currently Transmitting: Rover {}\n'.format(self.channel[-1].tx.radio_id))

        # Logically, this is the end of time slot.
        for receiver in self._rovers:
            if receiver.radio is None:
                pass
            elif receiver == transmitter:
                pass
            else:
                receiver.radio.receive(self)

        self.clear_channel()

        for controller in self._rovers:
            controller.apply_control(self)

        self._tn += 1
