from models.packet import *
from models.path_loss import *

PS = 8                              # Preamble symbol(s).
CRC = 1                             # Presence of cyclic redundancy check, default 1, meaning present.
HD = 1                              # Presence of header, default 1, meaning not present.
DE = 0                              # Presence of low data rate optimisation, default 0, meaning not present.
ANT_GAIN = 2.15                        # Antenna gain, in dBi.
ANT_HEIGHT = 0.17                   # Antenna height, in meter.
NF = 6                              # Noise figure, in dB.
SENSITIVITY = {125: {6: -122,
                     7: -124.5,
                     8: -127,
                     9: -129.5,
                     10: -132,
                     11: -134.5,
                     12: -137},
               250: {6: -119,
                     7: -121.5,
                     8: -124,
                     9: -126.5,
                     10: -129,
                     11: -131.5,
                     12: -134},
               500: {6: -116,
                     7: -118.5,
                     8: -121,
                     9: -123.5,
                     10: -126,
                     11: -128.5,
                     12: -131}}
# Sensitivity for each bandwidth & spreading factor, in dBm.


class Radio:
    """
    A radio class.
    """
    ps = PS     # The preamble symbol(s).
    crc = CRC   # The presence of CRC.
    hd = HD     # The presence of header.
    de = DE     # The presence of low data rate optimisation.
    ant_gain = ANT_GAIN         # The antenna gain， in dBi.
    ant_height = ANT_HEIGHT     # The antenna height, in meter.
    pl = 10                      # The length of payload, in byte.

    def __init__(self, rover, f, bw, sf, cr, tx_pw, t_slot=0.1):
        self._rover = rover                     # A rover object which the radio is attached to.
        self._radio_id = self._rover.rov_id     # Unique id for each radio, same as the rover's id.
        self._total_radios = 1                  # Number of all the radios in operation, including self.
        self._t_slot = t_slot                   # The length of time slot, in sec.
        # Time is represented by slot not second to avoid floating point number error.
        self._interval = 1                      # The time slot interval between transmissions.
        self._f = f                             # The carrier frequency, in MHz.
        self._bw = bw                           # The bandwidth, in KHz.
        self._sf = sf                           # The spreading factor.
        self._cr = cr                           # The coding rate.
        self._sensitivity = SENSITIVITY[self._bw][self._sf]    # The receiving sensitivity, in dBm.
        self._tx_pw = tx_pw                     # The transmitted power, in dBm.
        self._next_tx = 0 + self._radio_id - 1
        # The time slot for next transmission.
        # A transmission is only allowed to take place at the beginning of a time slot.
        # Each time slot only allows one transmission to happen.
        # Radios transmit messages in the order of their id number.
        self._num_transmitted = 0       # The number of transmitted packets.
        self._num_received = 0          # The number of received packets.
        self._num_discarded = 0         # The number of discarded packets.
        self._receiver_buffer = None    # The buffer to store the most recent packet received.
        self._neighbour_register = [None, None]
        # The memory to store the most packets received from neighbours (at most 2).

    @property
    def radio_id(self):
        return self._radio_id

    @property
    def total_radios(self):
        return self._total_radios

    @property
    def pos(self):
        return self._rover.pose

    @property
    def t_slot(self):
        return self._t_slot

    @property
    def interval(self):
        return self._interval

    @property
    def f(self):
        return self._f

    @property
    def bw(self):
        return self._bw

    @property
    def sf(self):
        return self._sf

    @property
    def cr(self):
        return self._cr

    @property
    def sensitivity(self):
        return self._sensitivity

    @property
    def tx_pw(self):
        return self._tx_pw

    @property
    def next_tx(self):
        return self._next_tx

    @property
    def num_tx(self):
        return self._num_transmitted

    @property
    def num_rx(self):
        return self._num_received

    @property
    def num_disc(self):
        return self._num_discarded

    @property
    def receiver_buffer(self):
        return self._receiver_buffer

    @property
    def neighbour_register(self):
        return self._neighbour_register

    def set_swarm_size(self, size):
        """
        Set total number of radios, including self.
        """
        self._total_radios = size
        self._neighbour_register = [None]*size

    def set_t_slot(self, new_t_slot):
        """
        Set slot length in second.
        """
        self._t_slot = new_t_slot

    def set_interval(self, new_interval=None):
        """
        Set interval length.
        """
        if new_interval is None:
            self._interval = self._total_radios
        else:
            self._interval = new_interval

    def set_f(self, new_f):
        """
        Set carrier frequency.
        """
        self._f = new_f

    def set_bw(self, new_bw):
        """
        Set bandwidth.
        """
        self._bw = new_bw

    def set_sf(self, new_sf):
        """
        Set spreading factor.
        """
        self._sf = new_sf

    def set_cr(self, new_cr):
        """
        Set coding rate.
        """
        self._cr = new_cr

    def set_sensitivity(self, new_sensitivity):
        """
        Set sensitivity.
        """
        self._sensitivity = new_sensitivity

    def set_txpw(self, new_txpw):
        """
        Set transmission power.
        """
        self._tx_pw = new_txpw

    def get_measurement(self):
        """
        Get the measured pose info ready for packet formation.
        """
        return self._rover.measurement
    
    def get_change_metric(self):
        """
        Get the measured pose info ready for packet formation.
        """
        return self._rover.change_metric[self._radio_id]     

    def transmit_change_metric(self, world):
        """
        Place a new transmission into the channel.
        """
        pose_msred = self.get_change_metric()
        payload = [self._radio_id, pose_msred[0], pose_msred[1]]
        packet = Packet(self, payload)
        self._num_transmitted += 1
        world.add_packet(packet)
        self._next_tx += self._interval

    def transmit_pos(self, world):
        """
        Place a new transmission into the channel.
        """
        tx_time = self._next_tx
        pose_msred = self.get_measurement()
        payload = [self._radio_id, tx_time, pose_msred[0], pose_msred[1]]
        packet = Packet(self, payload)
        self._num_transmitted += 1
        world.add_packet(packet)
        self._next_tx += self._interval

    def receive(self, world):
        """
        Receive a packet which can be successfully demodulated from the channel.
        """
        threshold = self._sensitivity
        if len(world.channel) > 0:
            packet = world.channel[-1]
            if self.rx_power(packet, world) >= threshold:
                self._num_received += 1
                self._receiver_buffer = packet
                self.update_neighbour_register()
            else:
                self._num_discarded += 1
        else:
            pass

    def rx_power(self, packet, world, max_dist=None):
        """
        Calculate the received power of an incoming packet from the channel.
        A maximum distance can be set beyond which the path loss is assumed -inf. This is
        optionally and should be used carefully.
        """
        path_loss = PathLoss(packet.tx, self, world, max_dist).total_loss()
        return packet.tx.tx_pw + packet.tx.ant_gain - path_loss + self.ant_gain

    def update_neighbour_register(self):
        """
        Update neighbour register.
        """
        packet = self._receiver_buffer
        tx_id = packet.tx.radio_id
        self._neighbour_register[tx_id - 1] = packet

    def reset_neighbour_register(self):
        """
        Reset neighbour register.
        """
        for i in range(self._total_radios):
            self._neighbour_register[i] = None

    def reset_buffer(self):
        """
        Reset receiver buffer.
        """
        self._receiver_buffer = None

    def airtime(self):
        """
        Calculate the airtime needed for each transmission.
        """
        t_symbol = pow(2, self._sf) / (self._bw * 1000)
        t_preamble = (self.ps + 4.25) * t_symbol
        term = int((8 * self.pl - 4 * self._sf + 28 + 16 * self.crc - 20 * self.hd) /
                   (4 * (self._sf - 2 * self.de))) * (1 / self._cr * 4)
        if term < 0:
            term = 0
        n_payload = 8 + term
        t_payload = n_payload * t_symbol
        return t_preamble + t_payload

    def actual_dc(self):
        """
        Calculate the actual duty cycle due to user-defined configuration.
        """
        return self.airtime() / (self.airtime() + self.actual_silent_time())

    def actual_silent_time(self):
        """
        Calculate the actual silent time due to use-defined configuration.
        """
        return self._interval * self._t_slot
