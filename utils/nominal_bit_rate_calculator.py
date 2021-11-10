BW = [125000, 250000, 500000]
SF = [6, 7, 8, 9, 10, 11, 12]
CR = [1, 2, 3, 4]


def main():
    """
    Calculate nominal bit rate.
    """
    for i in range(len(BW)):
        for j in range(len(SF)):
            for k in range(len(CR)):
                bw = BW[i]
                sf = SF[j]
                cr = CR[k]
                r_b = sf * (4 / (4 + cr)) / (2 ** sf) * bw
                print('BW = {} (Hz), SF = {}, CR = {}\nNominal bit rate = {} (bps)\n'.
                      format(str(bw), str(sf), str(cr), str(round(r_b))))


if __name__ == '__main__':
    main()
