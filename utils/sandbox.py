#
#Sandbox file where testing of different algorithms can be soft tested and ran in here
#
import numpy as np

def contourFormation():
    x = np.array([1,2,3,4,5, np.nan])
    w = np.array([1,2,3,4,5, 2])
    avg = np.average(x, weights=w)
    print(avg)

if __name__ == '__main__':
    contourFormation()