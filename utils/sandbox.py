#
#Sandbox file where testing of different algorithms can be soft tested and ran in here
#
import matplotlib.pyplot as plt

def contourFormation():
    x = [430475, 430925, 431375, 431825, 432275, 432725, 433175, 433625, 434075, 434525, 430454, 430908, 431364, 431812, 432264, 432707, 433164, 433611, 434061, 434510]
    y = [105005, 105005, 105005, 105005, 105005, 105005, 105005, 105005, 105005, 105005, 105531, 105531, 105531, 105531, 105531, 105531, 105531, 105531, 105531, 105531]
    z = [0.01717, 0.08678, 0.29248, 0.65747, 0.98575, 0.98575, 0.65747, 0.29248, 0.08678, 0.01717, 0.02643, 0.13784, 0.47821, 1.0827, 1.64394, 1.66521, 1.11841, 0.50585, 0.15199, 0.03058]

    fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 6))
        
    cmap = 'viridis'
    contf = ax.tricontourf(x, y, z, cmap=plt.get_cmap(cmap))
    # contf.set_clim(0, 150)
    plt.colorbar(contf, label='Measurment')
    plt.xlim([430000, 435000])
    plt.ylim([105000, 110000])
    plt.show()

if __name__ == '__main__':
    contourFormation()