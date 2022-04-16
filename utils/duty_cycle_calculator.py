bw = 125  # Bandwidth, in KHz
sf = 9    # Spreading factor
cr_num = 4
cr_den = 8
cr = cr_num / cr_den  # Coding rate
ps = 8                # Preamble symbol
pl = 10   # Payload, in byte
crc = 1  # Cyclic redundancy check
is_crc = {1: 'Yes',
          0: 'No'}
hd = 1  # Header
is_hd = {1: 'No',
         0: 'Yes'}
de = 0  # Low data rate optimisation
is_de = {1: 'Yes',
         0: 'No'}

len_interval = 120 # Steps to next transmission
dt = 0.1    # Simulation step time

def airtime_calc():
    """
    Calculate the airtime needed for each transmission.
    """
    t_symbol = (2 ** sf) / (bw * 1000)
    t_preamble = (ps + 4.25) * t_symbol
    term = int((8 * pl - 4 * sf + 28 + 16 * crc - 20 * hd) / (4 * (sf - 2 * de))) * (1 / cr * 4)
    if term < 0:
        term = 0
    n_payload = 8 + term
    t_payload = n_payload * t_symbol
    return t_preamble + t_payload


def main():
    """
    Calculate airtime & silent time of a packet given relevant parameters.
    """
    airtime = airtime_calc()
    silent_time =  len_interval * dt
    dc_calc = airtime / (airtime+silent_time)
    print('=' * 50)
    print('Bandwidth: {} (KHz)'.format(str(bw)))
    print('Spreading factor: {}'.format(str(sf)))
    print('Coding rate: {}/{}'.format(str(cr_num), str(cr_den)))
    print('Preamble: {} (symbol)'.format(str(ps)))
    print('Payload: {} (byte)'.format(str(pl)))
    print('Cyclic redundancy check (CRC): {}'.format(is_crc[crc]))
    print('Header: {}'.format(is_hd[hd]))
    print('Low data rate optimisation: {}'.format(is_de[de]))
    print('-' * 50)
    print('Airtime: {} (s)'.format(str(round(airtime, 6))))
    print('Silent time: {} (s)'.format(round(silent_time, 6)))
    print('Duty Cycle: {}%'.format(round(dc_calc*100, 6)))
    print('=' * 50)


if __name__ == '__main__':
    main()
