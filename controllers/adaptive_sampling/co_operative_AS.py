# when one rover reaches the designated total stop

import math
import numpy as np

"""
def linear_sampling_waypoints(rov):
    
    #Linear Sampler where sampling positions adjusted depending flux
    
    if(len(rov.measured_samples)> 3):
        rov._metric.append(abs(rov.measured_samples[-3]/rov.measured_samples[-2]))
        rov._metric.append(abs(rov.measured_samples[-2]/rov.measured_samples[-1]))
    
    if(rov._metric[1] > rov._metric[0]):
        rov._sample_dist += 50
    else:
        if(rov.sample_dist > 100):
            rov._sample_dist -= 50
"""

def gradient_calc(rov, i):
    """
    Calculation of gradient
    """
    dist_x = rov.measured_samples[-i+1][0] - rov.measured_samples[-i][0]
    dist_y = rov.measured_samples[-i+1][1] - rov.measured_samples[-i][1]
    dist = math.sqrt(dist_x**2 + dist_y**2)
    value = rov.measured_samples[-i+1][2] - rov.measured_samples[-i][2]
    return value / dist

def avg_pos(rov, i, xY):
    """
    Average position for gradient and second derivative metric
    """
    #change to for loop for adding the different x's as -2, -1, will caused IndexError
    return (rov.measured_samples[-i+2][xY] + rov.measured_samples[-i+1][xY] + rov.measured_samples[-i][xY]) / i

def euclidean_dist(x1, x2, y1, y2):
    dist_x =  x1 - x2 
    dist_y =  y1 - y2 
    return math.sqrt(dist_x**2 + dist_y**2)

def weight_neighbours(rov):
    """
    Weights the values of neighbouring position.
    """
    weights = []
    neighbour_metrics = []
    for i in range(len(rov.metric)):
        if(rov.metric[i][0] != 0 and i != rov.rov_id-1):
            dist = euclidean_dist(rov.metric[i][0], rov.metric[rov.rov_id-1][0], \
                                        rov.metric[i][1], rov.metric[rov.rov_id-1][1])
            weights.append(1/dist)
            neighbour_metrics.append(rov.metric[i][2])
    weights = np.array(weights)
    neighbour_metrics = np.array(neighbour_metrics)
    
    return weights, neighbour_metrics

def absolute_value(rov):
    """
    Absolute value of the sample measured
    """
    if(len(rov.measured_samples)>= rov.sample_metric_order + 1):
        rov.update_metric(rov.rov_id, rov.measured_samples[-1][0], rov.measured_samples[-1][1], abs(rov.measured_samples[-1][2]))

def first_derivative(rov):
    """
    Difference calc of the samples measured
    """
    sample_difference = []
    if(len(rov.measured_samples)>= rov.sample_metric_order + 1):
        sample_difference.append(gradient_calc(rov, 2))

        avg_x = avg_pos(rov, 2, 0)
        avg_y = avg_pos(rov, 2, 1)

        rov.update_metric(rov.rov_id, avg_x, avg_y, abs(sample_difference[0]))

def second_derivative(rov):
    """
    Difference calc of the samples measured
    """
    sample_difference = []
    if(len(rov.measured_samples)>= rov.sample_metric_order + 1):
        sample_difference.append(gradient_calc(rov, 3))
        sample_difference.append(gradient_calc(rov, 2))

        avg_x = avg_pos(rov, 3, 0)
        avg_y = avg_pos(rov, 3, 1)

        rov.update_metric(rov.rov_id, avg_x, avg_y, abs(sample_difference[1] - sample_difference[0]))

def update_sample_dist(rov, max_sample_dist, min_sample_dist):
    """
    Update sampling distance based on the metric of itself and neighbours
    """
    if(len(rov.measured_samples)>= rov.sample_metric_order):
        if(rov.metric.count([0, 0, 0]) < len(rov.metric)-1):
            w, neighbour_metrics = weight_neighbours(rov)
            neighbour_mean = np.average(neighbour_metrics, weights=w)

            neighbour_multiplier = 1 / (rov.K_sampler[0] * neighbour_mean + 1)
            rov_multiplier = 1 / (rov.K_sampler[0] * rov.metric[rov.rov_id-1][2] + 1)
            diff = rov_multiplier - neighbour_multiplier
    
            n = rov_multiplier + (rov.K_sampler[2] * diff)
        else:
            n = 1/(rov.K_sampler[0] * rov.metric[rov.rov_id-1][2] + 1)
        
        if(n < 0):
            n = 0
        
        temp_sample_dist = rov.avg_sample_dist * rov.K_sampler[1] * n

        if(temp_sample_dist < min_sample_dist):
            rov._sample_dist = min_sample_dist
        elif(temp_sample_dist > max_sample_dist):
            rov._sample_dist = max_sample_dist
        else:
            rov._sample_dist = temp_sample_dist

def co_op_sampler(rov, world, s_max, s_min):
    """
    Adaptive sampler sampling at regular intervals
    Linear or proportional adjustment of distance to next waypoint
    """
    if(len(rov.measured_samples)>0):
        travelled_dist = euclidean_dist(rov.pose[0], rov.measured_samples[-1][0], \
                                rov.pose[1], rov.measured_samples[-1][1])

    if((world.dt*world.tn == 0) or (len(rov.measured_samples) > 0 and travelled_dist >= rov.sample_dist)):    
        if(rov.num_samples < rov.max_num_samples and rov.is_sampling == False):
            rov._is_sampling = True
            rov._num_samples += 1
            print("Rover {} is taking a sample.".format(str(rov.rov_id)))

    neighbour_metrics = rov.get_neighbour_data()
    rov.connectivity_reset()                       #For mission connectivity.
    rov.neighbour_connectivity(neighbour_metrics)

    if(rov.req_sampling_steps == rov.sampling_steps_passed):
        p = rov.pose.copy()
        p[0], p[1] = round(p[0]), round(p[1])

        metric_measurement = round(world.sample_metric.sample(p[0], p[1]), 5)
        rov._sampling_steps_passed = 0
        rov.measured_samples.append([p[0], p[1], metric_measurement])

        if(rov.sample_metric_order==0):
            absolute_value(rov)
        elif(rov.sample_metric_order==1):
            first_derivative(rov)
        elif(rov.sample_metric_order==2):
            second_derivative(rov)
        
        if(neighbour_metrics.count(None)< len(neighbour_metrics)):
            for i in range(len(neighbour_metrics)):
                if(neighbour_metrics[i] is not None):
                    rov.update_metric(i+1, neighbour_metrics[i][1], neighbour_metrics[i][2], neighbour_metrics[i][0])

        update_sample_dist(rov, s_max, s_min)

        rov._is_sampling = False
        if(len(rov.measured_samples) >= rov.sample_metric_order):
            rov._transmit = True

    elif(rov.is_sampling):
        rov._sampling_steps_passed += 1