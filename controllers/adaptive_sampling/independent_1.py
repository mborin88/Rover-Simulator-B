#initial sample not taken
import math

def x_direction(value):
    if(value >= 0):
        return 1
    else:
        return -1

def ratio_speeds(rov):
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
    move_along_path(rov, v_max, v_min)
    p = rov._pose.copy()
    p[0], p[1] = round(p[0]), round(p[1])

    if((p[1] > rov.goal[1]-rov._goal_offset) and (rov._goal_index < len(rov._waypoints)-1) \
            or (world._dt*world._tn) == 0):   #if within offset of the y waypoint
        if(rov._num_samples > 0):
            rov._is_sampling = True
            rov._num_samples -= 1
            print("Rover {} is taking a sample.".format(str(rov._rov_id)))
    
    if(rov._sampling_steps == rov._sampling_steps_passed):
        metric_measurement = round(world._sample_metric.sample(p[0], p[1]), 5)
        rov._sampling_steps_passed = 0
        rov._measured_samples.append([p[0], p[1], metric_measurement])
        rov._is_sampling = False
    elif(rov._is_sampling):
        rov._sampling_steps_passed += 1