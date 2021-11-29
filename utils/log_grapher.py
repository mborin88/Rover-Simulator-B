import os

from numpy import promote_types

class ReplicaRover:
    """
    A Replica of the rover class.
    """
    def __init__(self, t_sample=0.1):
        #Include finding t_sample in log and setting
        self._t_sample = t_sample
        self._x_pose = []
        self._y_pose = []
        self._velocity = []

    @property
    def x_pose(self):
        return self._x_pose

    @property
    def y_pose(self):
        return self._y_pose

    @property
    def velocity(self):
        return self._velocity
    
    def set_t_sample(self, t_sample):
        self._t_sample = t_sample


def parse_file(rovers ,file):
    #do a check if rmse is added by number of hyphens
    
    rmse = []
    rmse_flag = False
    reached_data = False
    count = 0
    for line in file:
        parsed_data = []
        if line[:3] == '0.0': #Look for 0.0 as start of data file
            reached_data = True
        if reached_data:
            t_sample = float(line[:line.index('\t')])
            data_line = line[line.index('\t')+1:-2]
            rover_data_line = data_line.split('-')
            if(len(rover_data_line)==11):
                rmse_flag = True
            for rov_data in rover_data_line:
                parsed_data.append(rov_data.split(','))
            
            for i in range(len(rovers)-1):
                if(count==1):
                    rovers[i].set_t_sample(t_sample)
                rovers[i]._x_pose.append(float(parsed_data[i][0]))
                rovers[i]._y_pose.append(float(parsed_data[i][1]))
                rovers[i]._velocity.append(float(parsed_data[i][2]))

            if(rmse_flag):
                rmse.append(parsed_data[-1][0])
            count+=1

    return rovers, rmse


def velocity_line():
    pass

if __name__ == '__main__':
    map_name = 'SU30NE'
    control_policy = '3'
    file_name = 'SSS Raw Data, Simple Line Sweep, 2021-11-18 01-29-58' + '.txt'

    directory = "{}\\logs\\{}\\control_policy_{}\\".format(os.getcwd(), str(map_name), str(control_policy))

    Rovers = []
    for i in range(10):
        Rovers.append(ReplicaRover())
    try:
        log_file = open(directory + file_name, 'r')
    except: 
        print("File Not Found")
    data, rmse = parse_file(Rovers, log_file)
    
