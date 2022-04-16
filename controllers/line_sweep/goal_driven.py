
def move2goal(rov, v_max, v_min):
    """
    Move towards goal point.
    """
    controlled_object = rov.pos_measurement  # Controlled object is [x, y].
    rov.speed_controller.set_ref(rov._waypoints[-1])
    control_input = rov._speed_controller.execute2(controlled_object)
    if control_input > v_max:  # Control input saturation.
        rov._control[1] = v_max
    elif control_input < v_min:
        rov._control[1] = v_min
    else:
        rov._control[1] = control_input  # Assume changing linear velocity instantly.
    rov.update_speeds(0, rov._control[1])