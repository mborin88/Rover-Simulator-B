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
    p = rov._pose
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


def linear_sampling_waypoints(rov):
    """
    Linear Sampler where sampling positions adjusted depending flux
    """
    change_metric = []
    if(len(rov._measured_samples)> 3):
        change_metric.append(abs(rov._measured_samples[-3]/rov._measured_samples[-2]))
        change_metric.append(abs(rov._measured_samples[-2]/rov._measured_samples[-1]))
    
    if(change_metric[1] > change_metric[0]):
        rov._sample_dist += 50
    else:
        if(rov._sample_dist > 100):
            rov._sample_dist -= 50

def proportional_sampling_waypoints(rov):
    """
    Proporinal Sampler where sampling positions adjusted depending flux
    """
    K = 0
    change_metric = []
    if(len(rov._measured_samples)> 3):
        change_metric.append(abs(rov._measured_samples[-3]/rov._measured_samples[-2]))
        change_metric.append(abs(rov._measured_samples[-2]/rov._measured_samples[-1]))
    
    if(change_metric[1] > change_metric[0]):
        rov._sample_dist = rov._sample_dist *(change_metric[1]/change_metric[0])
    else:
        if(rov._sample_dist > 100):
            rov._sample_dist = rov._sample_dist * (change_metric[0]/change_metric[1])

def move_along_path(rov, v_max, v_min):
    """
    Move towards goal point.
    """
    controlled_object = rov.measurement  # Controlled object is [x, y].
    if(rov._pose[1] > rov.goal[1]-rov._goal_offset) \
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


def waypoint_sampler(rov, world, v_max, v_min):
    """
    Standard sampler sampling at regular intervals
    """
    if((world._dt*world._tn == 0) or \
            (len(rov._measured_samples) > 0 and rov._pose[1]-rov._measured_samples[-1][1] >= rov._sample_dist-rov._goal_offset)):
        if(rov._num_samples < rov._max_num_samples and rov._is_sampling == False):
            rov._is_sampling = True
            rov._num_samples += 1
            print("Rover {} is taking a sample.".format(str(rov._rov_id)))

    move_along_path(rov, v_max, v_min)

    if(rov._sampling_steps == rov._sampling_steps_passed):
        p = rov._pose.copy()
        p[0], p[1] = round(p[0]), round(p[1])
        metric_measurement = round(world._sample_metric.sample(p[0], p[1]), 5)
        rov._sampling_steps_passed = 0
        rov._measured_samples.append([p[0], p[1], metric_measurement])
        rov._is_sampling = False
    elif(rov._is_sampling):
        rov._sampling_steps_passed += 1

def adjusted_waypoint_sampler(rov, world, v_max, v_min):
    """
    Adaptive sampler sampling at regular intervals
    Linear or proportional adjustment of distance to next waypoint
    """

    if((world._dt*world._tn == 0) or \
            (len(rov._measured_samples) > 0 and rov._pose[1]-rov._measured_samples[-1][1] >= rov._sample_dist)):
        if(rov._num_samples < rov._max_num_samples and rov._is_sampling == False):
            rov._is_sampling = True
            rov._num_samples += 1
            print("Rover {} is taking a sample.".format(str(rov._rov_id)))

    move_along_path(rov, v_max, v_min)

    if(rov._sampling_steps == rov._sampling_steps_passed):
        p = rov._pose.copy()
        p[0], p[1] = round(p[0]), round(p[1])
        metric_measurement = round(world._sample_metric.sample(p[0], p[1]), 5)
        rov._sampling_steps_passed = 0
        rov._measured_samples.append([p[0], p[1], metric_measurement])
        rov._is_sampling = False
        proportional_sampling_waypoints(rov)
    elif(rov._is_sampling):
        rov._sampling_steps_passed += 1