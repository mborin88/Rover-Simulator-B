#For line sweep with only final goal no waypoints 

def legacy_move2goal(rov, v_max, v_min):
    """
    Move towards goal point.
    """
    controlled_object = rov.measurement  # Controlled object is [x, y].
    control_input = rov._speed_controller.execute(controlled_object)
    if control_input > v_max:  # Control input saturation.
        rov._control[0] = v_max
    elif control_input < v_min:
        rov._control[0] = v_min
    else:
        rov._control[0] = control_input  # Assume changing linear velocity instantly.