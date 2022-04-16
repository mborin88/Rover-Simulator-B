#For line sweep with only final goal no waypoints 

def move2goal(rov, v_max, v_min):
    """
    Move towards goal point.
    """
    controlled_object = rov.pos_measurement  # Controlled object is [x, y].
    # if(rov._pose[1] > rov.goal[1]-rov._goal_offset) \
    #     and (rov._goal_index < len(rov._waypoints)-1):   #if within offset of the y waypoint
    #     rov._goal_index += 1
    rov.speed_controller.set_ref(rov._waypoints[-1])
    control_input = rov._speed_controller.execute2(controlled_object)
    if control_input > v_max:  # Control input saturation.
        rov._control[0] = v_max
    elif control_input < v_min:
        rov._control[0] = v_min
    else:
        rov._control[0] = control_input  # Assume changing linear velocity instantly.