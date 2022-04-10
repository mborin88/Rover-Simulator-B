# now add to radio
# add to rover samples taken array [rov1_num_samples, rov2_num_samples]
# total samples taken for each rover
# when one rover reaches the designated total stop
# can we adaptive sample independently (ask danesh does y_concentration at another point affect another rovers sampling)
import math

def x_direction(value):
    """
    to retain x direction as trigonometry removes it.
    """
    if(value >= 0):
        return 1
    else:
        return -1

def ratio_speeds(rov):
    """
    Adjusts x and y speed so that rover is going in right direction
    """
    p = rov.pose
    target = rov.goal
    v = rov._control[2]
    x_diff = target[0] - p[0]
    y_diff = target[1] - p[1]
    try:
        angle = math.atan(y_diff/abs(x_diff))
    except ZeroDivisionError:
        angle = math.pi / 2
    rov._angle = angle
    
    rov._control[0] = round(x_direction(x_diff) * v * math.cos(angle), 3)
    rov._control[1] = round(v * math.sin(angle), 3)

def sample_difference(rov):
    """
    Proporinal Sampler where sampling positions adjusted depending flux
    """
    sample_difference = []
    if(len(rov.measured_samples)> 3):
        sample_difference.append(rov.measured_samples[-3][2] - rov.measured_samples[-2][2])
        sample_difference.append(rov.measured_samples[-2][2] - rov.measured_samples[-1][2])

        rov.update_change_metric(abs(sample_difference[1] - sample_difference[0]))        

def move_along_path(rov, v_max, v_min):
    """
    Move towards goal point.
    """
    controlled_object = rov.measurement  # Controlled object is [x, y].
    if(rov.pose[1] > rov.goal[1]-rov._goal_offset) \
        and (rov._goal_index < len(rov._waypoints)-1):   #if within offset of the y waypoint
        rov._goal_index += 1
        rov.speed_controller.set_ref(rov.goal)
    control_input = rov._speed_controller.execute2(controlled_object)

    if control_input > v_max:  # Control input saturation.
        rov._control[2] = v_max
    elif control_input < v_min:
        rov._control[2] = v_min
    else:
        rov._control[2] = control_input  # Assume changing linear velocity instantly.
    ratio_speeds(rov)


def proportional_adjustment_sampler(rov, world, v_max, v_min):
    """
    Adaptive sampler sampling at regular intervals
    Linear or proportional adjustment of distance to next waypoint
    """

    if((world._dt*world._tn == 0) or \
            (len(rov.measured_samples) > 0 and rov.pose[1]-rov.measured_samples[-1][1] >= rov.sample_dist)):
        if(rov.num_samples < rov.max_num_samples and rov.is_sampling == False):
            rov.is_sampling = True
            rov.num_samples += 1
            print("Rover {} is taking a sample.".format(str(rov._rov_id)))

    move_along_path(rov, v_max, v_min)

    if(rov.sampling_steps == rov.sampling_steps_passed):
        p = rov.pose.copy()
        p[0], p[1] = round(p[0]), round(p[1])
        metric_measurement = round(world._sample_metric.sample(p[0], p[1]), 5)
        rov.sampling_steps_passed = 0
        rov.measured_samples.append([p[0], p[1], metric_measurement])
        sample_difference(rov)
        rov.update_sample_dist()
        rov.is_sampling = False
    elif(rov.is_sampling):
        rov.sampling_steps_passed += 1