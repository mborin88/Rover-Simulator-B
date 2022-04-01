#Changes only y speed works with only final waypoint (old code)

from models.P_controller import *
import numpy as np

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
    for i in range(1,len(rov._all_control)):
        multiplier[i] = time_decay(rov, rov._steps_control_not_updated[i])
    rov._all_control = rov._all_control * multiplier


def passive_cooperation(rov, v_max, v_min):
    """
    Apply passive cooperative control, i.e. only adjust speed when neighbour(s)' info.
    Recieve all info scale old info to be less relevant. At certain time thershold old ignore old info.
    Slowly push all_control values that haven't been recieved to 0.
    """

    if(rov._pose[1] > rov.goal[1]-rov._goal_offset) \
        and (rov._goal_index < len(rov._waypoints)-1):   #if within offset of the y waypoint
        rov._goal_index += 1
    goal_driven_controller = PController(ref=rov._goal, gain=[0, 1e-3])
    controlled_object = rov.measurement
    control_input = goal_driven_controller.execute(controlled_object)
    
    if control_input > v_max:  # Control input saturation.
        p_control = v_max
    elif control_input < v_min:
        p_control = v_min
    else:
        p_control = control_input  # Assume changing linear velocity instantly. #velocity changes at end of cooperation 

    rov._all_control[0] = p_control

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
                    controlled_object = rov.measurement
                    rov._all_control[control_index] = rov._speed_controller.execute(controlled_object)
            scale_all_control(rov)
            control_input = weighted_control_calc(rov)
        else:
            scale_all_control(rov)
            control_input = weighted_control_calc(rov)
    else:
        control_input = p_control*1  # Take 100% portion of goal_driven control.
    
    #all incremented by 1
    
    if control_input > v_max:  # Control input saturation.
        rov._control[1] = v_max
    elif control_input < v_min:
        rov._control[1] = v_min
    else:
        rov._control[1] = control_input  # Assume changing linear velocity instantly.    

    rov._radio.reset_neighbour_register()
    rov._radio.reset_buffer()

def simple_passive_cooperation(rov, v_max, v_min):
    """
    Apply passive cooperative control, i.e. only adjust speed when neighbour(s)' info is received,
    otherwise do not apply any control effect.
    Start with P controller then only change speed when neighbour info recieved again.
    """

    if(rov._pose[1] > rov.goal[1]-rov._goal_offset) \
        and (rov._goal_index < len(rov._waypoints)-1):   #if within offset of the y waypoint
        rov._goal_index += 1
        
    goal_driven_controller = PController(ref=rov._goal, gain=[0, 1e-3])
    controlled_object = rov.measurement
    control_input = goal_driven_controller.execute(controlled_object)
    
    if control_input > v_max:  # Control input saturation.
        p_control = v_max
    elif control_input < v_min:
        p_control = v_min
    else:
        p_control = control_input  # Assume changing linear velocity instantly. #velocity changes at end of cooperation 

    rov._all_control[0] = p_control

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
                    rov._speed_controller.set_ref(pose)
                    controlled_object = rov.measurement
                    rov._all_control[control_index] = rov._speed_controller.execute(controlled_object)
                    rov._steps_control_not_updated[control_index] = 0 
            control_input = weighted_control_calc(rov)
            
        else:
            control_input = weighted_control_calc(rov)
    else:
        control_input = p_control*1  # Take 100% portion of goal_driven control.
    
    if control_input > v_max:  # Control input saturation. 
        rov._control[1] = v_max     #Only affects y position here as is the only one we care about
    elif control_input < v_min:
        rov._control[1] = v_min
    else:
        rov._control[1] = control_input  # Assume changing linear velocity instantly.    

    rov._radio.reset_neighbour_register()
    rov._radio.reset_buffer()