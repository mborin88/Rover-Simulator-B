# now add to radio
# add to rover samples taken array [rov1_num_samples, rov2_num_samples]
# total samples taken for each rover
# when one rover reaches the designated total stop
# can we adaptive sample independently (ask danesh does y_concentration at another point affect another rovers sampling)
import math

def gradient_calc(rov, i):
    dist_x = rov.measured_samples[-i+1][0] - rov.measured_samples[-i][0]
    dist_y = rov.measured_samples[-i+1][1] - rov.measured_samples[-i][1]
    dist = math.sqrt(dist_x**2 + dist_y**2)
    value = rov.measured_samples[-i+1][2] - rov.measured_samples[-i][2]
    return value / dist

def difference_calc(rov):
    """
    Difference calc of the samples measured
    """
    sample_difference = []
    if(len(rov.measured_samples)>= 3):
        sample_difference.append(gradient_calc(rov, 3))
        sample_difference.append(gradient_calc(rov, 2))

        rov.update_change_metric(abs(sample_difference[1] - sample_difference[0])) 
        #rov.update_change_metric(sample_difference[0])  

def update_sample_dist(rov, max_sample_dist, min_sample_dist):
    if(len(rov.measured_samples)>= 3):
        n = rov._change_metric[0]
        temp_sample_dist = rov.sample_dist * (rov._K_sampler[1]/(n+1))

        if(temp_sample_dist < min_sample_dist):
            rov._sample_dist = min_sample_dist
        elif(temp_sample_dist > max_sample_dist):
            rov._sample_dist = max_sample_dist
        else:
            rov._sample_dist = temp_sample_dist

def proportional_adjustment_sampler(rov, world, s_max, s_min):
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

        metric_measurement = round(world._sample_metric.sample(p[0], p[1]), 5)
        rov._sampling_steps_passed = 0
        rov._measured_samples.append([p[0], p[1], metric_measurement])

        difference_calc(rov)
        update_sample_dist(rov, s_max, s_min)
        rov._is_sampling = False

    elif(rov.is_sampling):
        rov._sampling_steps_passed += 1