import numpy as np
from math import *

from models.landcover_spec import *

# T_pc = 0.5  # Percentage of average year for which the predicted basic tx loss is not exceeded, set to 50% hereby.
# N_d1km50 = -45  # The refractivity gradient from the surface to 1 km above not exceeded for 50% of an average year.
R_e = 6371  # Earth radius, in km.
R_eff = (4 / 3) * R_e  # Effective earth radius, in km.
# C_50 = (157 + N_d1km50) / (157 * R_e)  # Effective earth curvature calculated from N_d1km50, in km^-1.
W_s = 27  # Street width, set to 27 if local info is unknown, in meter.
LIGHT_SPEED = 2.998 * pow(10, 8)  # Light speed, in m/s.


class PathLoss:
    """
    A class to model path loss for emitted signal during propagation.
    """

    def __init__(self, radio_tx, radio_rx, world, d_max=None):
        self._tx = radio_tx  # The transmitter, a radio object.
        self._rx = radio_rx  # The receiver, a radio object.
        self._tx_pos = self._tx.pos  # The transmitter position, (x, y).
        self._rx_pos = self._rx.pos  # The receiver position, (x, y).
        self._physical_world = world  # A world object where the channel is situated.
        self._d_max = d_max  # The maximum distance above which path loss is assumed -inf, in meter.
        self._d = sqrt((self._tx_pos[0] - self._rx_pos[0]) ** 2 +
                       (self._tx_pos[1] - self._rx_pos[1]) ** 2)  # Path length, in meter.
        self._h_tx = self._physical_world.terrain.get_data(self._tx_pos[0], self._tx_pos[1]) + \
                     self._tx.ant_height  # Tx height, in meter.
        self._h_rx = self._physical_world.terrain.get_data(self._rx_pos[0], self._rx_pos[1]) + \
                     self._rx.ant_height  # Rx height, in meter.
        self._tx_surrounding = LCM2015_NAME[int(self._physical_world.landcover.
                                                get_data(self._tx_pos[1],
                                                         self._tx_pos[0]))]  # The transmitter's surrounding category.
        self._rx_surrounding = LCM2015_NAME[int(self._physical_world.landcover.
                                                get_data(self._rx_pos[1],
                                                         self._rx_pos[0]))]  # The receiver's surrounding category.
                                                                            #landcover map inverted y <-> x swapped over purposedully
        self._f = self._tx.f  # Carrier wave frequency, in MHz.
        self._wavelength = LIGHT_SPEED / (self._f * pow(10, 6))  # Carrier wave wavelength, in meter.

    def __str__(self):
        return '=' * 50 + '\n' + \
               'Path Loss Model Specifications:\n' + '\n' + \
               'Propagation Distance: ' + (str(round(self._d, 6))) + ' (m)' + '\n' + \
               'Transmitter Coordinates: ' + str([round(self._tx_pos[0], 6), round(self._tx_pos[1], 6)]) + '\n' + \
               'Transmitter Altitude: ' + str(round(self._h_tx, 2)) + ' (m)' + '\n' + \
               'Transmitter Antenna Height: ' + str(self._tx.ant_height) + ' (m)' + '\n' + \
               "Transmitter's Surrounding: " + self._tx_surrounding + '\n' + \
               'Receiver Coordinates: ' + str([round(self._rx_pos[0], 6), round(self._rx_pos[1], 6)]) + '\n' + \
               'Receiver Altitude: ' + str(round(self._h_rx, 2)) + ' (m)' + '\n' + \
               'Receiver Antenna Height: ' + str(self._rx.ant_height) + ' (m)' + '\n' + \
               "Receiver's Surrounding: " + self._rx_surrounding + '\n' + \
               'Carrier Frequency: ' + str(self._f) + ' (MHz)' + '\n' + \
               '-' * 50 + '\n' + \
               'Free Space Loss: ' + str(round(self.free_space_loss(), 6)) + ' (dB)' + '\n' + \
               'Diffraction Loss: ' + str(round(self.diffraction_loss(), 6)) + ' (dB)' + '\n' + \
               'Clutter Loss: ' + str(round(self.clutter_loss(), 6)) + ' (dB)' + '\n' + \
               'Total Path Loss: ' + str(round(self.total_loss(), 6)) + ' (dB)' + '.' + '\n' + \
               '=' * 50

    def total_loss(self):
        """
        Calculate total path loss.
        Path loss of a propagation range larger than the maximum distance set by user can be
        assumed -inf. This is optionally and should be used carefully.
        """
        if self._d_max is None:
            return self.free_space_loss() + self.diffraction_loss() + self.clutter_loss()  # dB
        elif self._d < self._d_max:
            return self.free_space_loss() + self.diffraction_loss() + self.clutter_loss()  # dB
        else:
            return float('-inf')  # Negative infinity

    def free_space_loss(self):
        """
        Calculate basic free space loss.
        """
        return 20 * log10(self._d) + 20 * log10(self._f) - 27.55  # dB

    def diffraction_loss(self):
        """
        Calculate diffraction loss using principal knife edge method.
        """
        inter_profile = self.intermediate_profile()
        if inter_profile is not None:
            diff_param_p = self.principal_diff_param()
            diff_loss = self.knife_edge_loss(diff_param_p)
            return diff_loss
        else:
            return 0

    def clutter_loss(self):
        """
        Calculate clutter loss due to the surrounding where the transmitter or receiver is located.
        """
        f = self._f
        tx_ant_h = self._tx.ant_height
        rx_ant_h = self._rx.ant_height
        tx_surrounding = self._tx_surrounding
        rx_surrounding = self._rx_surrounding

        tx_clutter_type = None
        rx_clutter_type = None
        tx_loss = None
        rx_loss = None

        if (tx_surrounding == 'Default') \
                or (tx_surrounding == 'Arable and Horticulture') or (tx_surrounding == 'Improved Grassland') \
                or (tx_surrounding == 'Neutral Grassland') or (tx_surrounding == 'Calcareous Grassland') \
                or (tx_surrounding == 'Acid Grassland') or (tx_surrounding == 'Fen, Marsh and Swamp') \
                or (tx_surrounding == 'Heather') or (tx_surrounding == 'Heather Grassland') \
                or (tx_surrounding == 'Bog') or (tx_surrounding == 'Inland Rock') \
                or (tx_surrounding == 'Saltwater') or (tx_surrounding == 'Freshwater') \
                or (tx_surrounding == 'Supra-littoral Rock') or (tx_surrounding == 'Supra-littoral Sediment') \
                or (tx_surrounding == 'Littoral Rock') or (tx_surrounding == 'Littoral Sediment') \
                or (tx_surrounding == 'Saltmarsh'):
            tx_clutter_type = 1  # Open area
        elif (tx_surrounding == 'Broadleaved Woodland') or (tx_surrounding == 'Coniferous Woodland') \
                or (tx_surrounding == 'Urban') or (tx_surrounding == 'Suburban'):
            tx_clutter_type = 2  # Dense area

        if (rx_surrounding == 'Default') \
                or (rx_surrounding == 'Arable and Horticulture') or (rx_surrounding == 'Improved Grassland') \
                or (rx_surrounding == 'Neutral Grassland') or (rx_surrounding == 'Calcareous Grassland') \
                or (rx_surrounding == 'Acid Grassland') or (rx_surrounding == 'Fen, Marsh and Swamp') \
                or (rx_surrounding == 'Heather') or (rx_surrounding == 'Heather Grassland') \
                or (rx_surrounding == 'Bog') or (rx_surrounding == 'Inland Rock') \
                or (rx_surrounding == 'Saltwater') or (rx_surrounding == 'Freshwater') \
                or (rx_surrounding == 'Supra-littoral Rock') or (rx_surrounding == 'Supra-littoral Sediment') \
                or (rx_surrounding == 'Littoral Rock') or (rx_surrounding == 'Littoral Sediment') \
                or (rx_surrounding == 'Saltmarsh'):
            rx_clutter_type = 1  # Open area
        elif (rx_surrounding == 'Broadleaved Woodland') or (rx_surrounding == 'Coniferous Woodland') \
                or (rx_surrounding == 'Urban') or (rx_surrounding == 'Suburban'):
            rx_clutter_type = 2  # Dense area

        tx_clutter_height = LCM2015_CLUTTER_HEIGHT[tx_surrounding]
        if tx_ant_h >= tx_clutter_height:
            tx_loss = 0
        else:
            if tx_clutter_type == 1:  # Open area
                k_h2 = 21.8 + 6.2 * log10(f / 1000)
                tx_loss = -k_h2 * log10(tx_ant_h / tx_clutter_height)
            elif tx_clutter_type == 2:  # Dense area
                k_nu = 0.342 * sqrt(f / 1000)
                dh = tx_clutter_height - tx_ant_h
                theta = degrees(atan(dh / W_s))
                diff_param = k_nu * sqrt(dh * theta)
                tx_loss = self.knife_edge_loss(diff_param) - 6.03

        rx_clutter_height = LCM2015_CLUTTER_HEIGHT[rx_surrounding]
        if rx_ant_h >= rx_clutter_height:
            rx_loss = 0
        else:
            if rx_clutter_type == 1:  # Open area
                k_h2 = 21.8 + 6.2 * log10(f / 1000)
                rx_loss = -k_h2 * log10(rx_ant_h / rx_clutter_height)
            elif rx_clutter_type == 2:  # Dense area
                k_nu = 0.342 * sqrt(f / 1000)
                dh = rx_clutter_height - rx_ant_h
                theta = degrees(atan(dh / W_s))
                diff_param = k_nu * sqrt(dh * theta)
                rx_loss = self.knife_edge_loss(diff_param) - 6.03

        return tx_loss + rx_loss  # dB

    def terrain_profile(self):
        """
        Get a series of samples along the propagation path, including the tx and rx.
        """
        terrain = self._physical_world.terrain
        d = self._d
        d_sample = self._physical_world.terrain.resolution
        n_samples = int(d / d_sample)
        pos_x = np.linspace(self._tx_pos[0], self._rx_pos[0], n_samples)
        pos_y = np.linspace(self._tx_pos[1], self._rx_pos[1], n_samples)
        profile = []
        for i in range(n_samples):
            profile.append(terrain.get_data(pos_x[i], pos_y[i]))
        return profile

    def intermediate_profile(self):
        """
        Exclude the tx and rx from the terrain profile.
        """
        profile = self.terrain_profile()
        if len(profile) >= 3:
            return profile[1:-1]
        else:
            return None

    def principal_diff_param(self):
        """
        Calculate the diffraction parameter of the principal edge.
        """
        d = self._d / 1000
        d_sample = self._physical_world.terrain.resolution / 1000
        h_a = self._h_tx
        h_b = self._h_rx
        wavelength = self._wavelength
        inter_profile = self.intermediate_profile()
        n_samples = len(inter_profile)
        diff_params = []
        for i in range(n_samples):
            h_i = inter_profile[i]
            d_ai = (i + 1) * d_sample
            d_ib = d - d_ai
            h = h_i + (d_ai * d_ib) / (2 * R_eff) - (h_a * d_ib + h_b * d_ai) / d
            v_i = h * sqrt(2 * d / (wavelength * d_ai * d_ib))
            diff_params.append(v_i)
        principal_diff_param = max(diff_params)
        return principal_diff_param

    def knife_edge_loss(self, diff_param):
        """
        Calculate the knife edge diffraction loss given diffraction parameter.
        """
        if diff_param > -0.78:
            return 6.9 + 20 * log10(sqrt((diff_param - 0.1) ** 2 + 1) + diff_param - 0.1)  # dB
        else:
            return 0


'''
    def bullington_loss(self):
        """
        Calculate diffraction loss using Bullington method.
        """
        d = self._d / 1000
        los_slope = (self._h_rx - self._h_tx) / d  # Line of sight slope.
        inter_slopes = self.intermediate_slope()
        if inter_slopes is not None:
            peak_slope = max(inter_slopes)
            if peak_slope < los_slope:
                max_diff_param = self.max_diffraction_param()
                loss_dbka = self.knife_edge_loss(max_diff_param)
                loss_total = loss_dbka + (1 - exp(-loss_dbka / 6)) * (10 + 0.02 * d)
                return loss_total  # dB
            elif peak_slope >= los_slope:
                diff_param_bullington = self.diffraction_param_bullington()
                loss_dbka = self.knife_edge_loss(diff_param_bullington)
                loss_total = loss_dbka + (1 - exp(-loss_dbka / 6)) * (10 + 0.02 * d)
                return loss_total  # dB
        else:
            return 0

    def bullington_point(self):
        """
        Calculate the Bullington point.
        """
        d = self._d / 1000
        inter_slopes = self.intermediate_slope()
        if inter_slopes is not None:
            peak_slope = max(inter_slopes)
            inter_slopes_rx = self.intermediate_slope_rx()
            peak_slope_rx = max(inter_slopes_rx)
            d_bullington = (self._h_rx - self._h_tx + peak_slope_rx * d) / (peak_slope + peak_slope_rx)
            return d_bullington  # km
        else:
            return None
            
        def intermediate_slope(self):
        """
        Calculate the slope between the heights of intermediate points and the transmitter.
        """
        d = self._d / 1000
        inter_profile = self.intermediate_profile()
        if inter_profile is not None:
            inter_slopes = []
            d_sample = self._physical_world.terrain.resolution / 1000
            for i in range(len(inter_profile)):
                d_i = (i + 1) * d_sample
                inter_slope = (inter_profile[i] + 500 * C_50 * d_i * (d - d_i) - self._h_tx) / d_i
                inter_slopes.append(inter_slope)
            return inter_slopes
        else:
            return None

    def intermediate_slope_rx(self):
        """
        Similar to intermediate_slope() but calculates the slope to the receiver.
        """
        d = self._d / 1000
        inter_profile = self.intermediate_profile()
        if inter_profile is not None:
            inter_slopes_rx = []
            d_sample = self._physical_world.terrain.resolution / 1000
            for i in range(len(inter_profile)):
                d_i = (i + 1) * d_sample
                inter_slope = (inter_profile[i] + 500 * C_50 * d_i * (d - d_i) - self._h_rx) / d_i
                inter_slopes_rx.append(inter_slope)
            return inter_slopes_rx
        else:
            return None
            
    def max_diffraction_param(self):
        """
        Calculate the highest diffraction parameter along a given intermediate terrain profile.
        """
        d = self._d / 1000
        profile = self.intermediate_profile()
        if profile is not None:
            diff_params = []
            d_sample = self._physical_world.terrain.resolution
            for i in range(len(profile)):
                d_i = (i + 1) * d_sample / 1000
                diff_param = (profile[i] + 500 * C_50 * d_i * (d - d_i) -
                              (self._h_tx * (d - d_i) + self._h_rx * d_i) / d) * sqrt(0.002 * d
                                                                                      / (self._wavelength * d_i * (
                            d - d_i)))
                diff_params.append(diff_param)
            return max(diff_params)
        else:
            return None

    def diffraction_param_bullington(self):
        """
        Calculate diffraction parameter for the Bullington point.
        """
        d = self._d / 1000
        inter_slopes = self.intermediate_slope()
        if inter_slopes is not None:
            peak_slope = max(inter_slopes)
            d_bullington = self.bullington_point()
            diff_param_bullington = self._h_tx + peak_slope * d_bullington - (self._h_tx * (d - d_bullington) +
                                                                              self._h_rx * d_bullington) / d * sqrt(
                0.002 * d /
                (self._wavelength * d_bullington * (d - d_bullington)))
            return diff_param_bullington
        else:
            return None
'''
