# now add to radio
# add to rover samples taken array [rov1_num_samples, rov2_num_samples]
# total samples taken for each rover
# when one rover reaches the designated total stop
# can we adaptive sample independently (ask danesh does y_concentration at another point affect another rovers sampling
#change to euclidean distance

import math

# def linear_sampling_waypoints(rov):
#     """
#     Linear Sampler where sampling positions adjusted depending flux
#     """
#     if(len(rov.measured_samples)> 3):
#         rov._metric.append(abs(rov.measured_samples[-3]/rov.measured_samples[-2]))
#         rov._metric.append(abs(rov.measured_samples[-2]/rov.measured_samples[-1]))
    
#     if(rov._metric[1] > rov._metric[0]):
#         rov._sample_dist += 50
#     else:
#         if(rov.sample_dist > 100):
#             rov._sample_dist -= 50

def gradient_calc(rov, i):
    dist_x = rov.measured_samples[-i+1][0] - rov.measured_samples[-i][0]
    dist_y = rov.measured_samples[-i+1][1] - rov.measured_samples[-i][1]
    dist = math.sqrt(dist_x**2 + dist_y**2)
    value = rov.measured_samples[-i+1][2] - rov.measured_samples[-i][2]
    return value / dist

def avg_pos(rov, i, xY):
    #change to for loop for adding the different x's as -2, -1, will caused IndexError
    return (rov.measured_samples[-i+2][xY] + rov.measured_samples[-i+1][xY] + rov.measured_samples[-i][xY]) / i


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
    if(len(rov.measured_samples)>= 3):
        sample_difference.append(gradient_calc(rov, 3))
        sample_difference.append(gradient_calc(rov, 2))

        avg_x = avg_pos(rov, 3, 0)
        avg_y = avg_pos(rov, 3, 1)

        rov.update_metric(rov.rov_id, avg_x, avg_y, abs(sample_difference[1] - sample_difference[0]))

def update_sample_dist(rov, max_sample_dist, min_sample_dist):
    if(len(rov.measured_samples) >= rov.sample_metric_order):
        n = 1/(rov.K_sampler[0] * rov.metric[rov.rov_id-1][2] + 1)
        temp_sample_dist = rov.avg_sample_dist * rov.K_sampler[1] * n

        if(temp_sample_dist < min_sample_dist):
            rov._sample_dist = min_sample_dist
        elif(temp_sample_dist > max_sample_dist):
            rov._sample_dist = max_sample_dist
        else:
            rov._sample_dist = temp_sample_dist

def independent_sampler(rov, world, s_max, s_min):
    """
    Adaptive sampler sampling at regular intervals
    Linear or proportional adjustment of distance to next waypoint
    """

    if((world.dt*world.tn == 0) or \
            (len(rov.measured_samples) > 0 and rov.pose[1]-rov.measured_samples[-1][1] >= rov.sample_dist)):
        if(rov.num_samples < rov.max_num_samples and rov.is_sampling == False):
            rov._is_sampling = True
            rov._num_samples += 1
            print("Rover {} is taking a sample.".format(str(rov._rov_id)))

    if(rov.req_sampling_steps == rov.sampling_steps_passed):
        p = rov.pose.copy()
        p[0], p[1] = round(p[0]), round(p[1])

        metric_measurement = round(world.sample_metric.sample(p[0], p[1]), 5)
        rov._sampling_steps_passed = 0
        rov.measured_samples.append([p[0], p[1], metric_measurement])

        if(len(rov.measured_samples)> rov.sample_metric_order):
            if(rov.sample_metric_order==0):
                absolute_value(rov)
            elif(rov.sample_metric_order==1):
                first_derivative(rov)
            elif(rov.sample_metric_order==2):
                second_derivative(rov)

            update_sample_dist(rov, s_max, s_min)
            rov._transmit = True
        rov._is_sampling = False

    elif(rov.is_sampling):
        rov._sampling_steps_passed += 1