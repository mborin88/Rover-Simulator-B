from models.P_controller import *
import numpy as np
import warnings

def weighted_control_calc(rov):
    """
    Mean control of all the neighbouring velocities
    Then summed with the P controller Speed.
    """ 
    mean_control = 1
    if(mean_control):
        neighbour_mean = np.nanmean(rov._all_control[1:])
        return rov._all_control[0] + neighbour_mean

def time_decay(rov, value):
    """
    Reduces weight of adjustments speeds given by neighbouring rovers position
    """
    if(rov._decay_type == 'exp'):
        scale = (-1/np.exp(-value + rov._decay_zero_crossing)) + 1
    elif (rov._decay_type == 'quad'): 
        scale = (-1/(rov._decay_zero_crossing**2)) * (value-rov._decay_zero_crossing) * (value + rov._decay_zero_crossing)
    else:
        scale = 1

    if(scale <= 0):
        scale = 0

    return scale

def scale_all_control(rov):
    """
    Scale all control variable. 
    """ 
    multiplier = np.array([1.0]*len(rov._all_control))
    for i in range(1, len(rov._all_control)):
        multiplier[i] = time_decay(rov, rov._steps_control_not_updated[i])
    rov._all_control = rov._all_control * multiplier

def x_direction(value):
    """
    Points x speed in right direction
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
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            angle = math.atan(y_diff/abs(x_diff))
    except ZeroDivisionError:
        angle = math.pi / 2
    rov._angle = angle
    
    rov._control[0] = round(x_direction(x_diff) * v * math.cos(angle), 3)
    rov._control[1] = round(v * math.sin(angle), 3)

def advanced_simple_passive_cooperation(rov, world, v_max, v_min):
    """
    Apply passive cooperative control, i.e. only adjust speed when neighbour(s)' info is received,
    otherwise do not apply any control effect.
    Start with P controller then only change speed when neighbour info recieved again.
    """
    if(rov._pose[1] > rov.goal[1]-rov._goal_offset) \
        and (rov._goal_index < len(rov._waypoints)-1):   #if within offset of the y waypoint
        rov._goal_index += 1

    goal_driven_controller = PController(ref=rov._current_goal, gain=[1e-2, 1e-2])
    controlled_object = rov.pos_measurement
    control_input = goal_driven_controller.execute2(controlled_object)
    
    if control_input > v_max:  # Control input saturation.
        p_control = v_max
    elif control_input < v_min:
        p_control = v_min
    else:
        p_control = control_input  # Assume changing linear velocity instantly. #velocity changes at end of cooperation 

    rov._control[2] = p_control
    ratio_speeds(rov)
    rov._all_control[0] = p_control
    #x_dir = x_direction(rov._control[0])

    neighbour_poses = rov.get_neighbour_pose()
    if neighbour_poses.count(None) < rov._num_rovers:
        rov._initial_control = False

    rov.connectivity_reset()
    rov.neighbour_connectivity(neighbour_poses)

    for i in range(1, len(rov._steps_control_not_updated)):
        rov._steps_control_not_updated[i] += 1          # all incremented by 1

    if not rov._initial_control:
        # Adjust speed according to neighbour(s)' info.
        if neighbour_poses.count(None) < rov._num_rovers:
            control_index = 0
            for pose in neighbour_poses:
                control_index += 1
                if pose is not None:
                    rov._speed_controller.set_ref(pose)
                    controlled_object = rov.pos_measurement
                    rov._all_control[control_index] = rov._speed_controller.execute(controlled_object)
                    rov._steps_control_not_updated[control_index] = 0 
            control_input = weighted_control_calc(rov)
            
        else:
            control_input = weighted_control_calc(rov)
    else:
        control_input = p_control*1  # Take 100% portion of goal_driven control.
    
    if control_input > rov._control[1]:  # Control input saturation.
        rov._control[1] = rov._control[1]
    elif control_input < v_min:
        rov._control[1] = v_min
    else:
        rov._control[1] = control_input  # Assume changing linear velocity instantly.    

    rov.update_speeds(rov._control[0], rov._control[1])
    rov._transmit = True

    # if(rov.tx_status == 1):
    #     print('Previous transmission successful')
    # elif(rov.tx_status == 0):
    #     print('Queue transmissions as old one hasnt sent yet')
    # elif(rov.tx_status == -1):
    #     print('Resend old transmission as it failed. Or send new packet')

    rov._tx_buffer = [world.tn, controlled_object[0], controlled_object[1]]     
    rov._radio.reset_neighbour_register()
    rov._radio.reset_buffer()

def advanced_passive_cooperation(rov, world, v_max, v_min):
    """
    Apply passive cooperative control, i.e. only adjust speed when neighbour(s)' info.
    Recieve all info scale old info to be less relevant. At certain time thershold old ignore old info.
    Slowly push all_control values that haven't been recieved to 0.
    """
    if(rov._pose[1] > rov.goal[1]-rov._goal_offset) \
        and (rov._goal_index < len(rov._waypoints)-1):   #if within offset of the y waypoint
        rov._goal_index += 1

    goal_driven_controller = PController(ref=rov._current_goal, gain=[0, 1e-2])
    controlled_object = rov.pos_measurement
    control_input = goal_driven_controller.execute2(controlled_object)
    
    if control_input > v_max:  # Control input saturation.
        p_control = v_max
    elif control_input < v_min:
        p_control = v_min
    else:
        p_control = control_input  # Assume changing linear velocity instantly. #velocity changes at end of cooperation 

    rov._control[2] = p_control
    ratio_speeds(rov)

    rov._all_control[0] = rov._control[1]     #only want y speed to do line sweeping with

    neighbour_poses = rov.get_neighbour_pose()
    if neighbour_poses.count(None) < rov._num_rovers:
        rov._initial_control = False

    rov.connectivity_reset()
    rov.neighbour_connectivity(neighbour_poses)
    
    for i in range(1, len(rov._steps_control_not_updated)):
        rov._steps_control_not_updated[i] += 1 #all incremented by 1

    if not rov._initial_control:
        # Adjust speed according to neighbour(s)' info.
        if neighbour_poses.count(None) < rov._num_rovers:
            control_index = 0
            for pose in neighbour_poses:
                control_index += 1
                if pose is not None:
                    rov._steps_control_not_updated[control_index] = 0
                    rov._speed_controller.set_ref(pose)
                    rov._all_control[control_index] = rov._speed_controller.execute(controlled_object)
            scale_all_control(rov)
            control_input = weighted_control_calc(rov)
        else:
            scale_all_control(rov)
            control_input = weighted_control_calc(rov)
    else:
        control_input = rov._all_control[0] # Take 100% portion of goal_driven control.
    
    if control_input > rov._control[1]:  # Control input saturation. Only control the vy
        rov._control[1] = rov._control[1]
    elif control_input < 0:     #Test out with 0 next but could leave dead in the water
        rov._control[1] = 0
    else:
        rov._control[1] = control_input  # Assume changing linear velocity instantly.

    rov.update_speeds(rov._control[0], rov._control[1])
    rov._transmit = True
    # if(rov.tx_status == 1):
    #     print('Previous transmission successful')
    # elif(rov.tx_status == 0):
    #     print('Queue transmissions as old one hasnt sent yet')
    # elif(rov.tx_status == -1):
    #     print('Resend old transmission as it failed. Or send new packet')
    rov._tx_buffer = [world.tn, controlled_object[0], controlled_object[1]]  
    rov._radio.reset_neighbour_register()
    rov._radio.reset_buffer()